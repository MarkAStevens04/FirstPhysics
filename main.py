from typing import Optional
import global_

import pygame
# pip install pygame --pre
import time
import math


# POSSIBLE ERROR: during ball collision calculation, not sure what to do when xVelocity or yVelocity = 0

# Add a way of automatically checking if an error has occured (balls tp out of bounds or phase out of existence)
# Save the states of the balls immediately before this happens
# Then save those as test cases to run

def rainbowify(index: float, colors: list[tuple[int, int, int]]) -> tuple[int, int, int]:
    """
    takes a float 0 <= index <= 1
    and returns a color on the RGB map
    """
    if index == 1:
        index -= 0.0000001
    fracIndex = 1 / len(colors)
    pos = int(index // fracIndex)
    rem = index / fracIndex - pos
    # rem is how far the index is between the two given cells

    returnColor = [0, 0, 0]
    bottom = pos
    top = (bottom + 1) % len(colors)
    for channel in range(len(colors[bottom])):
        distance = colors[top][channel] - colors[bottom][channel]
        location = distance * rem + colors[bottom][channel]
        returnColor[channel] = location
    return returnColor[0], returnColor[1], returnColor[2]


def waitForFrame(start: time, end: time, fps: int, draw:bool, sc) -> None:
    """
    take a start and end time, along with a frame rate.
    Also take a screen to draw to if the last frame took longer than the FPS.

    If no screen is provided, will not do anything if the framerate
    is below what it should be
    Also take in information on whether the screen should have a
    shape drawn if the frame took too long to render
    """
    diff = end - start
    waitTime = 1 / fps - diff

    if waitTime > 0:
        time.sleep(waitTime)
    elif draw:
        pygame.draw.line(sc, (255, 0, 0), (SCREEN_X - 100, SCREEN_Y - 100), (SCREEN_X, SCREEN_Y), 10)


# This simulation performs the translation, then detects for collisions.
# An alternative way of doing this would be to maintain the object's old
# position, then perform a predictive calculation.
# Using vectors from the object's old position to the object's new position,
# we can detect intersection with other objects translation vectors, and
# perform collisions based on that.

# ANOTHER method could possibly be by adding equations to a list that
# the program should perform. Like, add an F=ma for obj1, as well as a
# tuple of (m1v11 + m2v12 = m1v21 + m1v22, KE1 + KE2 = KE1 + KE2). This
# would be really interesting, and I wonder how this would be implemented.
# This would most closely follow classical mechanics.

class Simulation:
    """
    Composite class which performs the simulation.

    == Attributes ==
    objects: a list containing all objects to be simulated
        [obj1, obj2, ... ]

    defaultForces: List of forces to be applied to all objects
        [force1, force2, ...]

    dampening: percentage of energy lost per collision

    precision: when to round to 0 for velocity calculations

    gravity: strength of gravity in the simulation

    bc: border color, (0 to 255, 0 to 255, 0 to 255)

    bs: buffer size. (size of borders in simulation)

    dc: debug color (color of debug lines)

    dt: delta time. The amount of time change that should be simulated

    == Methods ==
    step: perform a single iteration of the simulation
    """
    def __init__(self, x1: float, y1: float, x2: float, y2: float, damp: float, prec: float,
                 grav: float, bc: tuple[int, int, int], bs: float, dc: tuple[int, int, int],
                 dt: float):
        dampening = damp
        self.dampening = damp

        self.numCollisions = 0
        self.precision = prec

        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2

        # DEFAULT FORCES
        gravityForce = Force(0, grav)
        self.defaultForces = [gravityForce]
        self.grav = grav
        self.dampening = dampening
        self.dt = dt

        self.objects = []
        self.border = []
        self.border_color = bc
        self.buffer = bs
        self._border()
        self.debug_color = dc

    def _border(self):
        """
        Creates and adds a border to the simulation
        :return:
        """
        width1 = self.x2 - self.x1
        height1 = self.y2 - self.y1
        border_color = self.border_color
        thickness = self.buffer
        top = Box(self.x1, self.y1, width1, thickness, border_color)
        left = Box(self.x1, self.y1, thickness, height1, border_color)
        bottom = Box(self.x1, self.y2 - thickness, width1, thickness, border_color)
        right = Box(self.x2 - thickness, self.y1, thickness, height1, border_color)
        self.border.append(top)
        self.border.append(left)
        self.border.append(bottom)
        self.border.append(right)

    def _defaultForces(self):
        """
        Modify velocities from default forces
        :return: None
        """

        # gravity
        for obj in self.objects:
                # obj.calculateForce(defForce)
                obj.modifyVelocity([0, self.grav * self.dt])

    def _translate(self):
        """
        translates all objects simply based on their velocities
        :return:
        """
        for obj in self.objects:
            obj._translate()

    def _simpleDetectCollision(self, dynamic, static) -> bool:
        """
        Checks quickly if two objects are colliding
        :param dynamic: MUST BE A BALL
        :param static: MUST BE A BOX
        :return: True if intersecting, false otherwise
        """
        # NOTE! This method only works because the corners of the static object are not
        # in play. If the ball could intersect with corners, it would behave like a square.
        rad = dynamic.size
        ballX = dynamic.x
        ballY = dynamic.y

        ballLeft = ballX - rad
        ballRight = ballX + rad
        ballTop = ballY - rad
        ballBottom = ballY + rad

        if ballRight >= static.xLeft and ballLeft <= static.xRight:
            # x conditions are satisfied
            if ballBottom >= static.yTop and ballTop <= static.yBottom:
                # y conditions are satisfied
                return True

        return False

    def _simpleClosestEdge(self, dynamic, static) -> str:
        """
        Determines which component of the dynamic object's velocity should
        be reflected.

        = Pre-condition =
        dynamic and static are colliding

        :param dynamic: must be a Ball
        :param static: must be a box
        :return: either 'x' or 'y'
        """
        # Detects on which axis the ball has the minimum distance from the edge.
        # NOTE, this isn't completely accurate.
        # A better way to calculate this would be to re-trace steps based on the ball's velocity,
        # and detect which axis the ball was violating previously. Should only be 1.
        rad = dynamic.size
        ballX = dynamic.x
        ballY = dynamic.y

        ballLeft = ballX - rad
        ballRight = ballX + rad
        ballTop = ballY - rad
        ballBottom = ballY + rad

        # direction refers to the ball's perspective
        leftDistance = abs(ballLeft - static.xRight)
        rightDistance = abs(ballRight - static.xLeft)
        topDistance = abs(ballTop - static.yBottom)
        bottomDistance = abs(ballBottom - static.yTop)
        # print(f'ld: {leftDistance:.2f}, rd: {rightDistance:.2f}, tD: {topDistance:.2f}, bd: {bottomDistance:.2f}')

        xMin = min(leftDistance, rightDistance)
        yMin = min(topDistance, bottomDistance)
        if xMin <= yMin:
            return 'x'
        else:
            return 'y'

    def _teleportOutOfObject(self, dynamic, static, direction: str) -> None:
        """
        Modifies the dynamic object's position to get out of the static object.
        Velocity must already be updated from the collision.

        Updates precision

        :param dynamic: The moving object
        :param static: given static object
        :param direction: x or y, which direction should the ball move
        :return:
        """
        rad = dynamic.size
        ballX = dynamic.x
        ballY = dynamic.y
        buffer = 0

        ballLeft = ballX - rad
        ballRight = ballX + rad
        ballTop = ballY - rad
        ballBottom = ballY + rad

        dynamic._precisionVelocity(self.precision)

        xDistance, yDistance = 0, 0
        if direction == 'x':
            # calculate direction to move
            # then, calculate exactly how much to move
            if dynamic.velocity[0] > 0:
                # going to the right
                xDistance = -1 * (ballLeft - static.xRight) + buffer
            elif dynamic.velocity[0] < 0:
                # going to the left
                xDistance = -1 * (ballRight - static.xLeft) - buffer
            else:
                if abs(ballRight - static.xLeft) < abs(ballLeft - static.xRight):
                    yDistance = -1 * (ballRight - static.xLeft) - buffer
                else:
                    yDistance = -1 * (ballLeft - static.xRight) + buffer

        if direction == 'y':
            if dynamic.velocity[1] < 0:
                # going up
                yDistance = -1 * (ballBottom - static.yTop) - buffer
            elif dynamic.velocity[1] > 0:
                # going down
                yDistance = -1 * (ballTop - static.yBottom) + buffer
            else:
                if abs(ballBottom - static.yTop) < abs(ballTop - static.yBottom):
                    yDistance = -1 * (ballBottom - static.yTop) - buffer
                else:
                    yDistance = -1 * (ballTop - static.yBottom) + buffer

        # print(f'modifying ball positions: {xDistance:.2f}, {yDistance:.2f}')
        dynamic._modifyPosition([xDistance, yDistance])

    def _simpleCollision(self) -> None:
        """
        Simple collision between a dynamic and static object
        :return: None
        """
        # There is an edge case where the ball has so much velocity that it
        # literally phases through the object, lol.
        # This is a limitation of this calculation method.

        for obj in self.objects:
            for edge in self.border:
                if self._simpleDetectCollision(obj, edge):
                    # detect whether x or y should be reflected
                    side = self._simpleClosestEdge(obj, edge)
                    if side == 'x':
                        obj._elasticReflectionX()
                        obj._dampenX(self.dampening)
                        self.numCollisions += 1
                    elif side == 'y':
                        obj._elasticReflectionY()
                        obj._dampenY(self.dampening)

                    self._teleportOutOfObject(obj, edge, side)

    def _detectBallCollision(self, ball1, ball2) -> bool:
        """
        Detects if two balls are colliding
        :param ball1: ball object
        :param ball2: ball objectt
        :return: True or False depending on whether the balls are colliding.
        """
        # Because this performs the simulation, THEN checks for collisions, there
        # is again an edge case where the balls phase through one another.
        distance = ((ball1.x - ball2.x)**2 + (ball1.y - ball2.y)**2) ** 0.5
        length = ball1.size + ball2.size

        if length >= distance:
            return True
        else:
            return False

    def _ballUpdateVelocity(self, ball1, ball2) -> None:
        """
        Assumes two balls are colliding. Then, performs the energy
        and momentum equations to determine the new velocities of the
        balls.
        :param ball1:
        :param ball2:
        :return:
        """
        # assuming perfectly elastic collision, we find
        # m1 = ball1.mass
        # m2 = ball2.mass
        # v1xI = ball1.velocity[0]
        # v2xI = ball2.velocity[0]
        #
        # v1xF = ((m1 - m2) / (m1 + m2)) * v1xI + 2 * (m2 / (m1 + m2)) * v2xI
        # v2xF = 2 * (m1 / (m1 + m2)) * v1xI - ((m1 - m2) / (m1 + m2)) * v2xI
        #
        # v1yI = ball1.velocity[1]
        # v2yI = ball2.velocity[1]
        #
        # v1yF = ((m1 - m2) / (m1 + m2)) * v1yI + 2 * (m2 / (m1 + m2)) * v2yI
        # v2yF = 2 * (m1 / (m1 + m2)) * v1yI - ((m1 - m2) / (m1 + m2)) * v2yI
        #
        # ball1.setVelocity([v1xF, v1yF])
        # ball2.setVelocity([v2xF, v2yF])
        #
        # ball1._dampenX(self.dampening)
        # ball1._dampenY(self.dampening)
        # ball2._dampenX(self.dampening)
        # ball2._dampenY(self.dampening)

        # OUTLINE!
        # Get velocity vecotrs of two balls
        # Get vector connecting the center of two balls
        #       - to generalize this to more objects, get the normal vectors of the two surfaces
        # Tilt your axis, and get the balls' velocity vectors with respect to the direction of the
        #       collision. (get one vector parallel to the axis of collision, and one normal)
        # Perform the collision with respect to the parallel vectors, while keeping the normal
        #       vectors intact.
        # Take the new component velocity vectors and translate back into a single velocity vector
        # Un-tilt your axis
        # Expand into the original x and y axis
        # Set the new velocities

        # VARIABLES
        # dirVs# = direction of the initial velocity vector of ball #
        # magVs# = the magnitude of this vector
        # distance = distance between the two balls
        # dirCollision = angle with respect to the x-axis of the collision
        # dirBetween = angle between velocity and new axis
        # magVp# = magnitude of vector on parallel line for ball #
        # magVn# = magnitude of vector on normal line for ball #
        # magVsum# = magnitude of the new cumulative vector of the ball
        # dirFromTilt# = the theta between the tilted axis and the new veolocity vector
        # dirXY# = the theta between the new v vector & the xy plane
        # new## = magnitude of component vector for ball #



        magVs1 = ((ball1.velocity[0] ** 2) + (ball1.velocity[1] ** 2)) ** 0.5
        magVs2 = ((ball2.velocity[0] ** 2) + (ball2.velocity[1] ** 2)) ** 0.5

        # Handle edge cases when denominator is 0
        if ball1.velocity[0] == 0:
            dirVs1 = math.pi/2
        else:
            dirVs1 = math.atan(abs(ball1.velocity[1] / ball1.velocity[0]))

        if ball2.velocity[0] == 0:
            dirVs2 = math.pi/2
        else:
            dirVs2 = math.atan(abs(ball2.velocity[1] / ball2.velocity[0]))

        # b/c 0 < theta < pi/2 for tan inverse, we need to adjust quadrants.
        # I'm not exactly sure about the edge cases, but I figure it won't matter too much.
        if ball1.velocity[0] <= 0 and ball1.velocity[1] < 0:
            # going up and left. 2nd quadrant
            dirVs1 = math.pi - dirVs1
        elif ball1.velocity[0] < 0 and ball1.velocity[1] >= 0:
            # going down and left. 3rd quadrant
            dirVs1 += math.pi
        elif ball1.velocity[0] >= 0 and ball1.velocity[1] >= 0:
            # going down and right. 4th quadrant.
            dirVs1 = 2 * math.pi - dirVs1

        # now for the second ball
        if ball2.velocity[0] <= 0 and ball2.velocity[1] < 0:
            # going up and left. 2nd quadrant
            dirVs2 = math.pi - dirVs2
        elif ball2.velocity[0] < 0 and ball2.velocity[1] >= 0:
            # going down and left. 3rd quadrant
            dirVs2 += math.pi
        elif ball2.velocity[0] >= 0 and ball2.velocity[1] >= 0:
            # going down and right. 4th quadrant.
            dirVs2 = 2 * math.pi - dirVs2

        # print(f'b2 velocity: {ball2.velocity}')
        # print(f'dirVs1: {dirVs1 * 180 / math.pi:.2f}, dirVs2: {dirVs2 * 180 / math.pi:.2f}')
        distance = ((ball1.x - ball2.x) ** 2 + (ball1.y - ball2.y) ** 2) ** 0.5

        xDistance = ball2.x - ball1.x
        yDistance = ball1.y - ball2.y
        if xDistance == 0:
            dirCollision1 = math.pi/2
        else:
            dirCollision1 = math.atan(abs(yDistance / xDistance))

        if ball2.x <= ball1.x and ball2.y < ball1.y:
            # going up and left. 2nd quadrant
            dirCollision1 = math.pi - dirCollision1
        elif ball2.x < ball1.x and ball2.y >= ball1.y:
            # going down and left. 3rd quadrant
            dirCollision1 += math.pi
        elif ball2.x >= ball1.x and ball2.y > ball1.y:
            # going down and right. 4th quadrant.
            dirCollision1 = 2 * math.pi - dirCollision1


        dirCollision2 = dirCollision1

        # get parallel and normal lines in our tilted axis
        dirBetween1 = dirVs1 - dirCollision1
        dirBetween2 = dirVs2 - dirCollision2

        # print(f'collision: {dirCollision1 * 180 / math.pi:.2f}')
        # print(f'dir btwn. b1: {dirBetween1 * 180/math.pi:.2f}, b2: {dirBetween2 * 180/math.pi:.2f}')

        magVp1 = math.cos(dirBetween1) * magVs1
        magVp2 = math.cos(dirBetween2) * magVs2

        magVn1 = math.sin(dirBetween1) * magVs1
        magVn2 = math.sin(dirBetween2) * magVs2

        # perform collision in a single dimension
        m1 = ball1.mass
        m2 = ball2.mass

        magVp1new = ((m1 - m2) / (m1 + m2)) * magVp1 + ((2 * m2) / (m1 + m2)) * magVp2
        magVp2new = ((2 * m1) / (m1 + m2)) * magVp1 - ((m1 - m2) / (m1 + m2)) * magVp2

        # magVp1new *= -1
        # magVp2new *= -1

        # print(f'b1 initial v: {magVp1:.2f}, final v: {magVp1new:.2f}')
        # print(f'b2 initial v: {magVp2:.2f}, final v: {magVp2new:.2f}')
        # print(f'b1 normal v: {magVn1:.2f}')
        # print(f'b2 normal v: {magVn2:.2f}')

        magVp1new *= ((1-self.dampening) ** (0.5))
        magVp2new *= ((1-self.dampening) ** (0.5))


        # now convert back to original axises
        magVsum1 = ((magVp1new ** 2) + (magVn1 ** 2)) ** 0.5
        magVsum2 = ((magVp2new ** 2) + (magVn2 ** 2)) ** 0.5



        if magVsum1 == 0:
            dirFromTilt1 = 0
        else:
            dirFromTilt1 = abs(math.asin(magVn1 / magVsum1))
            # print(f'theta before adj. b1: {dirFromTilt1 * 180 / math.pi:.2f}')

            if magVp1new > 0 and magVn1 >= 0:
                # quad 1
                pass
            elif magVp1new <= 0 and magVn1 > 0:
                # quad 2
                dirFromTilt1 = math.pi - dirFromTilt1
            elif magVp1new < 0 and magVn1 <= 0:
                # quad 3
                dirFromTilt1 = math.pi + dirFromTilt1
            else:
                # quad 4
                dirFromTilt1 = math.pi * 2 - dirFromTilt1

        if magVsum2 == 0:
            dirFromTilt2 = 0
        else:
            dirFromTilt2 = abs(math.asin(magVn2 / magVsum2))
            # print(f'theta before adj. b2: {dirFromTilt2 * 180 / math.pi:.2f}')

            if magVp2new > 0 and magVn2 >= 0:
                # quad 1
                dirFromTilt2 = dirFromTilt2
            elif magVp2new <= 0 and magVn2 > 0:
                # quad 2
                dirFromTilt2 = math.pi - dirFromTilt2
            elif magVp2new < 0 and magVn2 <= 0:
                # quad 3
                dirFromTilt2 = math.pi + dirFromTilt2
            else:
                # quad 4
                dirFromTilt2 = math.pi * 2 - dirFromTilt2

        #
        # if magVsum1 == 0 or magVsum2 == 0:
        #     if magVsum1 == 0:
        #         dirFromTilt1 = 0
        #     if magVsum2 == 0:
        #         dirFromTilt2 = 0
        # else:
        #     dirFromTilt1 = abs(math.asin(magVn1 / magVsum1))
        #     dirFromTilt2 = abs(math.asin(magVn2 / magVsum2))
        #     # if asin <0, it means the vector is in quad 2 or 4.
        #     # start at 0 if parallel is > 0, and start at pi if parallel is < 0
        #
        #     print(f'theta before adj. b1: {dirFromTilt1 * 180 / math.pi:.2f}, b2: {dirFromTilt2 * 180 / math.pi:.2f}')
        #
        #     if magVp1new > 0 and magVn1 >= 0:
        #         # quad 1
        #         pass
        #     elif magVp1new <= 0 and magVn1 > 0:
        #         # quad 2
        #         dirFromTilt1 = math.pi - dirFromTilt1
        #     elif magVp1new < 0 and magVn1 <= 0:
        #         # quad 3
        #         dirFromTilt1 = math.pi + dirFromTilt1
        #     else:
        #         # quad 4
        #         dirFromTilt1 = math.pi * 2 - dirFromTilt1
        #
        #
        #     if magVp2new > 0 and magVn2 >= 0:
        #         # quad 1
        #         dirFromTilt2 = dirFromTilt2
        #     elif magVp2new <= 0 and magVn2 > 0:
        #         # quad 2
        #         dirFromTilt2 = math.pi - dirFromTilt2
        #     elif magVp2new < 0 and magVn2 <= 0:
        #         # quad 3
        #         dirFromTilt2 = math.pi + dirFromTilt2
        #     else:
        #         # quad 4
        #         dirFromTilt2 = math.pi * 2 - dirFromTilt2


        # possibly need to adjust dirCollision here?
        # POSSIBLE ERROR POSSIBLE ERROR POSSIBLE ERROR POSSIBLE ERROR POSSIBLE ERROR
        # just do a +math.pi on all dirCollision when used for v2
        dirXY1 = dirCollision1 + dirFromTilt1
        dirXY2 = dirCollision2 + dirFromTilt2

        # print(f'Direction from tilt b1: {dirFromTilt1 * 180/math.pi:.2f}, b2: {dirFromTilt2 * 180/math.pi:.2f}')
        # print(f'ball 1 new direction: {(dirXY1 * 180 / math.pi):.2f}')
        # print(f'ball 2 new direction: {(dirXY2 * 180 / math.pi):.2f}')

        newX1 = math.cos(dirXY1) * magVsum1
        newX2 = math.cos(dirXY2) * magVsum2

        newY1 = math.sin(dirXY1) * magVsum1 * -1
        newY2 = math.sin(dirXY2) * magVsum2 * -1

        # print(f'new velocity b1: {newX1:.2f}, {newY1:.2f}')
        # print(f'new velocity b2: {newX2:.2f}, {newY2:.2f}')

        ball1.setVelocity([newX1, newY1])
        ball2.setVelocity([newX2, newY2])


        # print()

        # debug the velocity vectors
        ball1.debugLine.setDirection(dirCollision1)
        ball2.debugLine.setDirection(math.pi + dirCollision1)
        # ball1.debugLine.setDirection(dirVs1)
        # ball2.debugLine.setDirection(dirVs2)

    def _singleTP(self, ball1, ball2) -> None:
        """
        teleports two balls along their axis of intersection to
        no longer be intersecting.

        = pre-condition =
        Two balls must be intersecting
        :param obj1:
        :param obj2:
        :return:
        """
        # REMOVE BUFFER REMOVE BUFFER REMOVE BUFFER
        buffer = 0
        distance = ((ball1.x - ball2.x) ** 2 + (ball1.y - ball2.y) ** 2) ** 0.5 - buffer
        xDistance = ball2.x - ball1.x
        yDistance = ball1.y - ball2.y
        # print('teleporting')

        if xDistance == 0:
            dirCollision1 = math.pi / 2
        else:
            dirCollision1 = math.atan(abs(yDistance / xDistance))

        if xDistance == 0 and yDistance == 0:
            # print('balls phased inside each other?')
            raise NotImplementedError
        elif ball2.x <= ball1.x and ball2.y < ball1.y:
            # going up and left. 2nd quadrant
            dirCollision1 = math.pi - dirCollision1
        elif ball2.x < ball1.x and ball2.y >= ball1.y:
            # going down and left. 3rd quadrant
            dirCollision1 += math.pi
        elif ball2.x >= ball1.x and ball2.y > ball1.y:
            # going down and right. 4th quadrant.
            dirCollision1 = 2 * math.pi - dirCollision1

        # print(f'new dir collision: {dirCollision1 * 180 / math.pi:.2f}')
        # xDistance = ball2.x - ball1.x
        # yDistance = ball1.y - ball2.y
        # if xDistance == 0:
        #     dirCollision1 = math.pi / 2
        # else:
        #     dirCollision1 = math.atan(abs(yDistance / xDistance))
        #
        # if ball2.x <= ball1.x and ball2.y < ball1.y:
        #     # going up and left. 2nd quadrant
        #     dirCollision1 = math.pi - dirCollision1
        # elif ball2.x < ball1.x and ball2.y >= ball1.y:
        #     # going down and left. 3rd quadrant
        #     dirCollision1 += math.pi
        # elif ball2.x >= ball1.x and ball2.y > ball1.y:
        #     # going down and right. 4th quadrant.
        #     dirCollision1 = 2 * math.pi - dirCollision1

        totMoveDistance = (distance - (ball1.size + ball2.size)) * -1
        m1 = ball2.mass / (ball1.mass + ball2.mass) * totMoveDistance
        m2 = ball1.mass / (ball1.mass + ball2.mass) * totMoveDistance

        moveX1 = math.cos(dirCollision1) * m1
        moveY1 = math.sin(dirCollision1) * m1

        moveX2 = math.cos(dirCollision1) * m2
        moveY2 = math.sin(dirCollision1) * m2


        # I want to weight the balls' movement by their weights


        # print(f'moving by: {moveX:.2f}, {moveY:.2f}')
        # print(f'initial pos. b1: {ball1.x:.2f}, {ball1.y:.2f}, b2: {ball2.x:.2f}, {ball2.y:.2f}')

        ball1._modifyPosition([-1 * moveX1, moveY1])
        ball2._modifyPosition([moveX2, -1 * moveY2])

        # print(f'final   pos. b1: {ball1.x:.2f}, {ball1.y:.2f}, b2: {ball2.x:.2f}, {ball2.y:.2f}')

        # print(f'modifying the balls. Move distance: {moveDistance:.2f}')
        # print(f'x: {moveX:.2f}, y: {moveY:.2f}')
        # print()

    def _twoBallCollisions(self) -> None:
        """
        Performs collisions for all balls in the simulation
        :return:
        """
        # this is incredibly inefficient. This checks every pair of balls.
        # In the future, I could do chunking to make this easier, only detecting
        # balls within certain pre-defined chunks.
        # another option is to save all positions of all balls in a sorted list, then go
        # through that list as if it were chunking
        for index1 in range(len(self.objects)):
            for index2 in range(index1 + 1, len(self.objects)):
                # every pair of balls
                # print(index1, index2)
                colliding = self._detectBallCollision(self.objects[index1], self.objects[index2])
                if colliding:
                    self.objects[index1]._precisionVelocity(self.precision)
                    self.objects[index2]._precisionVelocity(self.precision)

                    self._ballUpdateVelocity(self.objects[index1], self.objects[index2])
                    self.numCollisions += 1

        max_reps = 10
        i = 1
        solved = False
        while i <= max_reps and not solved:
            solved = True
            for index1 in range(len(self.objects)):
                for index2 in range(index1 + 1, len(self.objects)):
                    # every pair of balls
                    colliding = self._detectBallCollision(self.objects[index1], self.objects[index2])
                    if colliding:
                        solved = False
                        self._singleTP(self.objects[index1], self.objects[index2])

            i += 1

    def step(self):
        """
        Simulates all objects
        :return:
        """
        # calculate new velocities from fores
        # translate objects
        # detect collisions
        # re-calculate velocities

        # calculate new V from forces
        self._defaultForces()

        # translate all objects
        self._translate()

        # collision handling
        self._simpleCollision()
        self._twoBallCollisions()
        # print('simulated! ')
        # print(f'initial pos. b1: {self.objects[0].x:.2f}, {self.objects[0].y:.2f}, b2: {self.objects[1].x:.2f}, {self.objects[1].y:.2f}')
        # print(f'final   pos. b1: {self.objects[0].x:.2f}, {self.objects[0].y:.2f}, b2: {self.objects[1].x:.2f}, {self.objects[1].y:.2f}')
        #
        # print()

    def addObjectToSimulation(self, ball) -> None:
        """
        adds a ball to the simulation
        :param ball:
        :return:
        """
        self.objects.append(ball)

    def addBall(self, index: int):
        """
        quick and dirty add a ball to the screen
        :param index:
        :return:
        """

        if index == 0:
            sizeBall = 50
            offset = 1
            defaultMass = 1
            # xStart = self.x1 + BUFFER + sizeBall + offset
            # yStart = self.y1 + BUFFER + sizeBall + offset
            xStart = SCREEN_X - 300
            yStart = SCREEN_Y - 200
            # yStart = SCREEN_Y / 2 + 50
            newBall = Ball(xStart, yStart, sizeBall, BALL_COLOR, defaultMass, self.debug_color, self.dt)
            self.objects.append(newBall)
            # newBall.modifyVelocity([-25, 0])
            # newBall.modifyVelocity([-20, 0])
            newBall.modifyVelocity([0, 0])
            newLine = DebugLine(50, 60, (0, 0))
            newLine.setFollowObject(newBall)

        elif index >= 1:
            sizeBall = 50
            offset = 1
            defaultMass = 1
            # xStart = self.x1 + BUFFER + sizeBall + offset
            # yStart = self.y1 + BUFFER + sizeBall + offset
            xStart = 300
            yStart = BUFFER + sizeBall
            # yStart = SCREEN_Y / 2 - 100
            newBall = Ball(xStart, yStart, sizeBall, BALL_COLOR, defaultMass, self.debug_color, self.dt)
            self.objects.append(newBall)
            # newBall.modifyVelocity([15, 0])
            newBall.modifyVelocity([0, 0])
            # newBall.modifyVelocity([15, 5])

    def addBallAtPosition(self, position) -> None:
        """
        Spawns a ball at a specific position.
        :param position:
        :return:
        """
        sizeBall = 50
        defaultMass = 1
        xStart = position[0]
        yStart = position[1]
        newBall = Ball(xStart, yStart, sizeBall, BALL_COLOR, defaultMass, self.debug_color, self.dt)
        self.objects.append(newBall)
        newBall.modifyVelocity([0, 0])
        newLine = DebugLine(50, 60, (0, 0))
        newLine.setFollowObject(newBall)

    def draw(self, sc, index: float):
        """
        draws all objects to the screen. Also rainbowifys the balls
        :param sc: screen to draw on
        :param index: float from 0 to 1 of where the simulation is
        :return:
        """
        for edge in self.border:
            edge.draw(sc)

        if global_.COLORIFY:
            rainbowColors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]

        for obj in self.objects:
            if global_.COLORIFY:
                obj.setColor(rainbowify(index, rainbowColors))
            obj.draw(sc)


class Force:
    """
    Tracks attributes about forces that should be applied
    == Attributes ==
    x: Strength in x direction
    y: Strength in y direction
    """
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y


class Ball:
    """
    Stores attributes of a single ball in the physics engine

    == Attributes ==
    x: x position, float
    y: y position, float
    size: size of the ball, float

    mass: Scalar of mass of ball

    forces: list of all forces applied on the ball
    velocity: 2 dimensional vector of velocity
        [xVelocity, yVelocity]

    debugLine: small line used for debugging

    dt: amount of time needed to simulate
    """
    def __init__(self, x: float, y: float, size: float, color: tuple[int, int, int],
                 mass: float, debug_color: tuple[int, int, int], dt: float):
        self.x = x
        self.y = y
        self.size  = size
        self.color = color
        self.mass = mass
        self.forces = []
        self.velocity = [0, 0]
        self.dt = dt
        self.debug_color = debug_color
        self.debugLine = DebugLine(50, 60, (self.x, self.y))
        self.debugLine.setFollowObject(self)

    def _translate(self):
        """
        Utilizes the ball's current velocity to translate the ball.
        Doesn't calculate collisions
        :return:
        """
        self.x = self.x + (self.velocity[0] * self.dt)
        self.y = self.y + (self.velocity[1] * self.dt)

    def _modifyPosition(self, coords: list[float, float]) -> None:
        """
        Modifies the ball's position based on the given coordinates
        :param coords:
        :return:
        """
        self.x += coords[0]
        self.y += coords[1]

    def modifyVelocity(self, newVelocity: list[float, float]):
        self.velocity[0] = self.velocity[0] + newVelocity[0]
        self.velocity[1] = self.velocity[1] + newVelocity[1]

    def calculateForce(self, f1):
        """
        Modify velocity from force
        :param f1: Force to calculate
        :return:
        """
        xVelocityChange = f1.x / self.mass
        yVelocityChange = f1.y / self.mass
        self.modifyVelocity([xVelocityChange, yVelocityChange])

    def setVelocity(self, newVelocity: list[float, float]):
        self.velocity[0] = newVelocity[0]
        self.velocity[1] = newVelocity[1]

    def _elasticReflectionX(self) -> None:
        """
        Changes the X velocity based on physics of an
        elastic reflection
        :return:
        """
        self.velocity[0] = (-1) * self.velocity[0]

    def _elasticReflectionY(self) -> None:
        """
        Changes the Y velocity based on physics of an
        elastic reflection
        :return:
        """
        self.velocity[1] = (-1) * self.velocity[1]

    def _dampenX(self, percentage: float) -> None:
        """
        Modifies the X velocity based on the energy of the ball
        :param percentage: Dampening percentage
        :return: None
        """
        # after reducing the equation KE new = percentage * KE old
        # the equation for the new velocity is just
        # vNew = vOld * (percentage ** (0.5))
        vNewX = self.velocity[0] * ((1-percentage) ** (0.5))
        self.setVelocity([vNewX, self.velocity[1]])

    def _dampenY(self, percentage: float) -> None:
        """
        Modifies the X velocity based on the energy of the ball
        :param percentage: Dampening percentage
        :return: None
        """
        # after reducing the equation KE new = percentage * KE old
        # the equation for the new velocity is just
        # vNew = vOld * (percentage ** (0.5))
        vNewY = self.velocity[1] * ((1-percentage) ** (0.5))
        self.setVelocity([self.velocity[0], vNewY])

    def _precisionVelocity(self, precision: float) -> None:
        """
        sets the balls velocity to 0 if the velocity is within the required precision
        :param precision:
        :return:
        """
        if -1 * precision <= self.velocity[0] <= precision:
            self.velocity[0] = 0
        if -1 * precision <= self.velocity[1] <= precision:
            self.velocity[1] = 0

    def setColor(self, newColor: tuple[int, int, int]) -> None:
        """
        Modifies the ball's color to that which was given
        :param newColor: new color
        :return: None
        """
        self.color = newColor

    def draw(self, sc):
        """
        Draws the ball on the given surface
        :param sc:
        :return:
        """
        pygame.draw.circle(sc, self.color, (self.x, self.y), self.size)
        if global_.DEBUG:
            self.debugLine.draw(sc)


class Box:
    """
    A box object! Immovable.
    """
    def __init__(self, x: float, y: float, width: float, height: float, color: tuple[int, int, int]):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.color = color

        self.xLeft, self.yTop, self.xRight, self.yBottom = self.edgePositions()

    def _translate(self):
        """
        Stationary boxes don't move!
        :return:
        """
        self.x = self.x
        self.y = self.y

    def edgePositions(self) -> tuple[float, float, float, float]:
        """
        Returns the positions of edges for this box.
        xLeft side, yTop side, xRight side, yBottom side
        :return:
        """
        xLeft = self.x
        yTop = self.y
        xRight = self.x + self.width
        yBottom = self.y + self.height
        return xLeft, yTop, xRight, yBottom

    def draw(self, sc):
        """
        Draws the box to the screen
        :param sc:
        :return:
        """
        bottomRightX = self.x + self.width
        bottomRightY = self.y + self.height
        pygame.draw.rect(sc, self.color, (self.x, self.y, bottomRightX, bottomRightY))


class DebugLine:
    """
    Line to assist in debugging!
    """
    def __init__(self, length: float, theta: float, start: tuple[float, float]):
        self.length = length
        self.theta = theta
        self.start = start

        self.radians = self.theta * (2 * math.pi / 360)
        self.follow = None

    def setFollowObject(self, obj):
        """
        sets the follow object. The start position of this line will always be that
        of the object.
        :param obj:
        :return:
        """
        self.follow = obj
        self.color = self.follow.debug_color

    def _getStartPosition(self):
        """
        Updates the start position to reflect that of the followed object
        :return:
        """
        self.start = (self.follow.x, self.follow.y)

    def _getDirection(self):
        """
        Takes the object's velocity and converts it into the
        line's direction
        :return:
        """
        self.radians = math.atan((self.follow.velocity[1])/(self.follow.velocity[0]))

    def setDirection(self, rad: float):
        """
        Takes radian value and updates the attribute of this line
        :param theta:
        :return:
        """
        self.radians = rad

    def draw(self, sc):
        """
        draws the debug line to the screen
        :param sc:
        :return:
        """
        self._getStartPosition()
        # self._getDirection()
        x = (math.cos(self.radians) * self.length) + self.start[0]
        y = -1 * (math.sin(self.radians) * self.length) + self.start[1]

        end = (x, y)
        pygame.draw.line(sc, self.color, self.start, end, 3)
        pygame.draw.line(sc, self.color, self.start, (x, self.start[1]))

        self.theta = self.radians * 180 / math.pi

        img = global_.my_font.render(f'{self.theta:.0f}', True, (255, 255, 255))
        sc.blit(img, (self.start[0] - 20, self.start[1] - 20))


class FPSobj:
    """
    an object which tracks everything for FPS

    sc is the screen
    """
    def __init__(self, sc):
        self.draw_size = 30
        self.numFrames = 200
        self.updateFrequency = 50

        start = time.time()
        self.avg = 0
        self.frameTimeStamps = [start]
        self.currentFrame = 0
        self.sc = sc

        self.draw_font = pygame.font.SysFont(global_.font, self.draw_size)

    def calcAverages(self) -> float:
        """
        Calculate avg time difference for previous numFrames.

        :return: fps from averages
        """
        # first, remove any frames which should not be included.
        excess = len(self.frameTimeStamps) - self.numFrames
        print()
        if excess > 0:
            self.frameTimeStamps = self.frameTimeStamps[excess:]

        # now, obtain the average time
        print(f'last timestamp: {self.frameTimeStamps[-1:]}')
        avgFramesPerSecond = (len(self.frameTimeStamps) - 1) / (self.frameTimeStamps[-1:][0] - self.frameTimeStamps[0])
        return avgFramesPerSecond

    def logAndDraw(self):
        self.frameTimeStamps.append(time.time())
        self.currentFrame += 1
        if self.currentFrame >= self.updateFrequency:
            self.avg = self.calcAverages()
            self.currentFrame = 0
        self.draw()

    def draw(self):
        img = global_.my_font.render(f'{self.avg:.0f}', True, (255, 255, 255))
        self.sc.blit(img, (SCREEN_X - BUFFER - 2 * self.draw_size, BUFFER + 2 * self.draw_size))


def main_loop():
    sim1 = Simulation(0, 0, SCREEN_X, SCREEN_Y, 0.01, precision, gravity, BORDER_COLOR, BUFFER, DEBUG_COLOR, dt)
    # test_accuracy(sim1)
    # sim1.addBall(0)
    # sim1.addBall(1)
    # a = 2
    a = 0

    i = 0
    i2 = 0
    numBalls = 1000

    running = True
    while running:

        start = time.time()
        screen.fill(BACKGROUND)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.MOUSEBUTTONUP:
                if a < numBalls:
                    pos = pygame.mouse.get_pos()
                    sim1.addBallAtPosition(pos)
                    a += 1

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    if a < numBalls:
                        pos = pygame.mouse.get_pos()
                        sim1.addBallAtPosition(pos)
                        a += 1

        i += 0.002
        i2 += 0.01
        if i2 >= 1:
            i2 = 0
            print(sim1.numCollisions)
            print(f'a: {a}')

        if i >= 1:
            i = 0

        sim1.step()
        sim1.draw(screen, i)

        frame_shower.logAndDraw()

        # Ensure constant framerate
        end = time.time()
        waitForFrame(start, end, FPS, global_.SHOW_FPS_BEHIND, screen)
        s2 = time.time()
        pygame.display.update()


if __name__ == '__main__':
    # Technical
    SCREEN_X = 1390
    SCREEN_Y = 700
    FPS = 120
    global_.DEBUG = True
    global_.COLORIFY = True

    # Aesthetic
    BACKGROUND = (0, 0, 0)
    # BALL_COLOR = (255, 255, 255)
    BALL_COLOR = (100, 100, 100)
    BORDER_COLOR = (255, 100, 100)
    BUFFER = 50
    pygame.font.init()
    font = 'Avenir'
    my_font = pygame.font.SysFont(font, 15)
    DEBUG_COLOR = (100, 255, 100)

    # Initialize screen
    screen = pygame.display.set_mode((SCREEN_X, SCREEN_Y), pygame.SCALED)
    pygame.display.set_caption("TEST TEST TEST")

    # For the simulation
    stepSize = 0.5
    true_gravity = 1000
    precision = 0.00000000001

    # smaller calculations
    FPSsim = 60
    gravity = true_gravity
    dt = 1 / FPSsim * stepSize

    frame_shower = FPSobj(screen)
    main_loop()
