import time

import pygame.font

from main import *
import global_

visualize = True
show_fps = True
global_.COLORIFY = False
global_.DEBUG = False

# SETUP
SCREEN_X = 1390
SCREEN_Y = 700
BUFFER = 50
screen = pygame.display.set_mode((SCREEN_X, SCREEN_Y), pygame.SCALED)
pygame.display.set_caption("Test cases")
FPS = 100
DEBUG = True
stepSize = 0.1
true_gravity = 1000
precision = 0.00000000001
FPSsim = 60
gravity = true_gravity
dt = 1/FPSsim * stepSize

# Aesthetic
BACKGROUND = (0, 0, 0)
BALL_COLOR = (100, 100, 100)
BORDER_COLOR = (255, 100, 100)
DEBUG_COLOR = (100, 255, 100)


class simulation_assertions:
    """
    Handles some basic assertions about simulations.
    Just run the init and the final function, and all the
    assertions about a given simulation should be taken care of.

    = Pre-conditions =
    Simulation must already be set up. No new objects or information
    can be passed in.
    """
    def __init__(self, simulation):
        """
        Tests:

        - Number of balls preserved
        - All balls within boundaries
        Conservation of momentum (each axis)
        Conservation of energy (ke)
        """
        self.accuracy_needed = 0.0001
        self.sim = simulation
        self.init_objs = self._get_initial_objs()

        # create a lower and upper bound for momentum in each axis
        mom_x = abs(self._momentum_in_direction('x'))
        self.init_momentum_x = [mom_x - (self.accuracy_needed * mom_x) - precision, mom_x + (self.accuracy_needed * mom_x) + precision]

        mom_y = abs(self._momentum_in_direction('y'))
        self.init_momentum_y = [mom_y - (self.accuracy_needed * mom_y) - precision, mom_y + (self.accuracy_needed * mom_y) + precision]

        # create a lower and upper bound for energy in each axis
        ene_x = abs(self._energy_in_direction('x'))
        self.init_energy_x = [ene_x - (self.accuracy_needed * ene_x) - precision,
                                ene_x + (self.accuracy_needed * ene_x) + precision]

        ene_y = abs(self._energy_in_direction('y'))
        self.init_energy_y = [ene_y - (self.accuracy_needed * ene_y) - precision,
                                ene_y + (self.accuracy_needed * ene_y) + precision]


    def _get_initial_objs(self) -> set:
        """
        returns the current number of balls in the simulation
        :return:
        """
        to_return = set()
        for obj in self.sim.objects:
            to_return.add(obj)
        return to_return

    def _check_bounds(self):
        """
        Ensures all objects are within the simulation's boundaries
        :return:
        """
        within_bounds = True
        min_x = BUFFER
        max_x = SCREEN_X - BUFFER
        min_y = BUFFER
        max_y = SCREEN_X - BUFFER
        for obj in self.sim.objects:
            if not(obj.x <= max_x - obj.size):
                print('object out of bounds')
                assert obj.x <= max_x - obj.size
            if not(obj.x >= min_x + obj.size):
                print('object out of bounds')
                assert obj.x >= min_x + obj.size
            if not(obj.y <= max_y - obj.size):
                print('object out of bounds')
                assert obj.y <= max_y - obj.size
            if not(obj.y >= min_y + obj.size):
                print('object out of bounds')
                assert obj.y >= min_y + obj.size

    def _momentum_in_direction(self, axis: str) -> float:
        """
        Sums the total momentum in a given direction.

        * NOTE *
        momentum is NOT conserved in the y-axis because we do not track
        gravity.
        :param axis: 'x' or 'y'
        :return: float of total momentum on that axis
        """
        momentum = 0
        if axis == 'x':
            i = 0
        elif axis == 'y':
            i = 1
        else:
            print('INVALID AXIS')
            raise SyntaxError

        for obj in self.sim.objects:
            momentum += obj.velocity[i] * obj.mass

        return momentum

    def _KE_single_calculation(self, obj, index) -> float:
        """
        Performs a single KE calculation
        :param object:
        :return:
        """
        return (1 / 2) * obj.mass * (obj.velocity[index] ** 2)

    def _GravPotential_single_calculation(self, obj) -> float:
        """
        Performs a single gravitational potential energy calculation
        :param object:
        :return:
        """
        distance_to_bottom = self.sim.y2 - self.sim.buffer - obj.y
        return obj.mass * self.sim.grav * distance_to_bottom

    def _energy_in_direction(self, axis: str) -> float:
        """
        Calculates the energy for a given axis.
        ONLY cares about KE and potential energy due to gravity

        :param axis: 'x' or 'y'
        :return: float corresponding to energy at the given simulation state.
        """
        energy = 0
        if axis == 'x':
            i = 0
        elif axis == 'y':
            i = 1
            # account for gravitational potential energy
            for obj in self.sim.objects:
                energy += self._GravPotential_single_calculation(obj)
        else:
            print('INVALID AXIS')
            raise SyntaxError

        for obj in self.sim.objects:
            # calculate and add the kinetic energy
            energy += self._KE_single_calculation(obj, i)

        return energy

    def object_assertions(self) -> None:
        """
        Performs assertions for all objects in the simulation.
        Checks all objects still exist, and that all objects are within the bounds
        :return:
        """
        # number of objects in simulation is same at end as in beginning
        if len(self.init_objs) != len(self._get_initial_objs()):
            print('objects have disappeared from simulation')
            assert len(self.init_objs) == len(self._get_initial_objs()), 'objects have disappeared from the simulation'

        # all objects exist in the bounds
        self._check_bounds()

    def momentum_assertion(self, axis: str) -> None:
        """
        Performs momentum assertions on the given axis
        :param axis: either 'x' or 'y'
        :return: None
        """
        if axis == 'x':
            if abs(self._momentum_in_direction('x')) < self.init_momentum_x[0] or abs(self._momentum_in_direction('x')) > self.init_momentum_x[1]:
                print(f'momentum not conserved in x axis!')
                print(f'actual: {abs(self._momentum_in_direction("x")):.2f}')
                print(f"expect: {self.init_momentum_x[0]:.2f}, {self.init_momentum_x[1]:.2f}")

                assert abs(self._momentum_in_direction('x')) >= self.init_momentum_x[0]
                assert abs(self._momentum_in_direction('x')) <= self.init_momentum_x[1]
        elif axis == 'y':
            if abs(self._momentum_in_direction('y')) < self.init_momentum_y[0] or abs(
                    self._momentum_in_direction('y')) > \
                    self.init_momentum_y[1]:
                print('momentum not conserved in y axis!')
                print(f'actual: {abs(self._momentum_in_direction("y")):.2f}')
                print(f"expect: {self.init_momentum_y[0]:.2f}, {self.init_momentum_y[1]:.2f}")

                assert abs(self._momentum_in_direction('y')) >= self.init_momentum_y[0]
                assert abs(self._momentum_in_direction('y')) <= self.init_momentum_y[1]

    def energy_assertion(self, axis: str) -> None:
        """
        Performs energy assertions on the given axis
        :param axis: either 'x' or 'y'
        :return: None
        """
        if axis == 'x':
            if abs(self._energy_in_direction('x')) < self.init_energy_x[0] or abs(self._energy_in_direction('x')) > \
                    self.init_energy_x[1]:
                print('energy not conserved in x axis!')
                print(f'actual: {abs(self._energy_in_direction("x")):.2f}')
                print(f"expect: {self.init_energy_x[0]:.2f}, {self.init_energy_x[1]:.2f}")

                assert abs(self._energy_in_direction('x')) >= self.init_energy_x[0]
                assert abs(self._energy_in_direction('x')) <= self.init_energy_x[1]
        elif axis == 'y':
            if abs(self._energy_in_direction('y')) < self.init_energy_y[0] or abs(self._energy_in_direction('y')) > \
                    self.init_energy_y[1]:
                print('energy not conserved in y axis!')
                print(f'actual: {abs(self._energy_in_direction("y")):.2f}')
                print(f"expect: {self.init_energy_y[0]:.2f}, {self.init_energy_y[1]:.2f}")

                assert abs(self._energy_in_direction('y')) >= self.init_energy_y[0]
                assert abs(self._energy_in_direction('y')) <= self.init_energy_y[1]

    def run_all_assertions(self):
        """
        Runs many tests to ensure the simulation behaves as expected
        :return:
        """
        print()
        self.object_assertions()
        self.momentum_assertion('x')
        self.energy_assertion('x')
        self.energy_assertion('y')

class FPSobj:
    """
    an object which tracks everything for FPS

    sc is the screen
    """
    def __init__(self, sc):
        self.draw_size = 30
        self.numFrames = 200
        self.updateFrequency = 50
        self.x = SCREEN_X - BUFFER - 2 * self.draw_size
        self.y = BUFFER + self.draw_size

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
        if excess > 0:
            self.frameTimeStamps = self.frameTimeStamps[excess:]

        # now, obtain the average time
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
        self.sc.blit(img, (self.x, self.y))

# initializing stuff
if show_fps:
    frame_shower = FPSobj(screen)

def test_tests():
    assert 1 == 1


def test_accuracy() -> None:
    """
    Checks accuracy of pi physics calculation
    """
    sim1 = Simulation(0, 0, SCREEN_X, SCREEN_Y, 0, precision, gravity, BORDER_COLOR, BUFFER, DEBUG_COLOR, dt)
    ACCURACY = 2

    sizeBall = 25

    largerMass = 100 ** ACCURACY
    smallerMass = 1

    xStart1 = 300 + sizeBall + 50
    xStart2 = 300

    yStart1 = SCREEN_Y - BUFFER - sizeBall
    yStart2 = SCREEN_Y - BUFFER - sizeBall

    ball1 = Ball(xStart1, yStart1, sizeBall, (180, 180, 180), largerMass, DEBUG_COLOR, dt)
    ball2 = Ball(xStart2, yStart2, sizeBall, BALL_COLOR, smallerMass, DEBUG_COLOR, dt)

    sim1.addObjectToSimulation(ball1)
    sim1.addObjectToSimulation(ball2)

    ball1.modifyVelocity([-25, 0])
    ball2.modifyVelocity([0, 0])

    # newLine1 = DebugLine(50, 60, (0, 0))
    # newLine1.setFollowObject(ball1)
    # newLine2 = DebugLine(50, 60, (0, 0))
    # newLine2.setFollowObject(ball2)

    base_assertions = simulation_assertions(sim1)
    running = True
    while ball1.x <= SCREEN_X - BUFFER - (2 * sizeBall) and running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        if visualize:
            screen.fill(BACKGROUND)

        sim1.step()

        if visualize:
            sim1.draw(screen, 0)
            if show_fps:
                frame_shower.logAndDraw()
            pygame.display.update()



    base_assertions.run_all_assertions()
    # add extra collision if ball is moving to the right.
    # print 'invalid test' if magnitude of ball2 is greater than that of ball 1
    # at the end of the simulation

    # run base assertions before this if/then
    if abs(ball2.velocity[0]) >= abs(ball1.velocity[0]):
        assert 1 == 0, f'ball moving too fast at end of simulation, invalid test! Num collisions: {sim1.numCollisions}'
    elif ball2.velocity[0] < 0:
        sim1.numCollisions += 1

    pi = 3.14159265358979
    calculated_pi = int(pi * (10 ** ACCURACY))
    print()
    print(f'num collisions: {sim1.numCollisions}')
    assert sim1.numCollisions >= calculated_pi - 1
    assert sim1.numCollisions <= calculated_pi + 1


def test_single_drop() -> None:
    """
    Just a template for tests
    :return:
    """
    sim1 = Simulation(0, 0, SCREEN_X, SCREEN_Y, 0, precision, gravity, BORDER_COLOR, BUFFER, DEBUG_COLOR, dt)

    sizeBall = 50

    mass1 = 1

    xStart1 = SCREEN_X / 2
    yStart1 = BUFFER + 100

    ball1 = Ball(xStart1, yStart1, sizeBall, BALL_COLOR, mass1, DEBUG_COLOR, dt)

    sim1.addObjectToSimulation(ball1)

    ball1.modifyVelocity([0, 0])

    num_steps = 100 * (1 / stepSize)
    curr_step = 1

    i2 = 1
    i2max = 5

    base_assertions = simulation_assertions(sim1)
    running = True
    print()
    print('KE         |  Grav Potential  |  Total  E   |   y Pos  |  y Velocity')
    while running and curr_step <= num_steps:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        if visualize:
            screen.fill(BACKGROUND)

        sim1.step()

        GravPE = base_assertions._GravPotential_single_calculation(ball1)
        KinE = base_assertions._KE_single_calculation(ball1, 1)

        if curr_step % (20 * (1/stepSize)) == 0:
            # print(f'KE            : {KinE:010.2f}')
            # print(f'Grav Potential:           {GravPE:010.2f}')
            # print(f'total KE      :                  {KinE + GravPE:010.2f}')
            print(f'{KinE:010.2f}    {GravPE:010.2f}        {KinE + GravPE:010.2f}     {ball1.y:05.2f}    {ball1.velocity[1]:.2f}')
            if i2 == 1:
                startingTotalE = KinE + GravPE
            if i2 == 5:
                finishingTotalE = KinE + GravPE
                print(f'difference is: {finishingTotalE - startingTotalE:.2f}')
            i2 += 1



        if visualize:
            sim1.draw(screen, 0)
            if show_fps:
                frame_shower.logAndDraw()
            pygame.display.update()

        curr_step += 1


    base_assertions.run_all_assertions()


def test_high_speed() -> None:
    """
    Just a template for tests
    :return:
    """
    sim1 = Simulation(0, 0, SCREEN_X, SCREEN_Y, 0, precision, gravity, BORDER_COLOR, BUFFER, DEBUG_COLOR, dt)

    sizeBall = 25

    mass1 = 10
    mass2 = 3

    xStart1 = SCREEN_X - BUFFER - 100
    xStart2 = BUFFER + 100

    yStart1 = BUFFER + 100
    yStart2 = SCREEN_Y - BUFFER - 100

    ball1 = Ball(xStart1, yStart1, sizeBall, BALL_COLOR, mass1, DEBUG_COLOR, dt)
    ball2 = Ball(xStart2, yStart2, sizeBall, BALL_COLOR, mass2, DEBUG_COLOR, dt)

    sim1.addObjectToSimulation(ball1)
    sim1.addObjectToSimulation(ball2)

    ball1.modifyVelocity([-2000, 4000])
    ball2.modifyVelocity([4000, 3000])

    # newLine1 = DebugLine(50, 60, (0, 0))
    # newLine1.setFollowObject(ball1)
    # newLine2 = DebugLine(50, 60, (0, 0))
    # newLine2.setFollowObject(ball2)

    # number of secs sim should run
    num_steps = 300 * (1/stepSize)
    curr_step = 1

    base_assertions = simulation_assertions(sim1)
    running = True
    while running and curr_step <= num_steps:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        if visualize:
            screen.fill(BACKGROUND)

        sim1.step()

        if visualize:
            sim1.draw(screen, 0)
            if show_fps:
                frame_shower.logAndDraw()
            pygame.display.update()

        curr_step += 1


    base_assertions.run_all_assertions()


def test_vertical_drop() -> None:
    """
    Tests edge cases for balls bouncing directly on the axis
    :param simulation:
    :return:
    """
    sim1 = Simulation(0, 0, SCREEN_X, SCREEN_Y, 0, precision, gravity, BORDER_COLOR, BUFFER, DEBUG_COLOR, dt)

    sizeBall = 50
    defaultMass = 1

    xStart = SCREEN_X / 2
    yStart = SCREEN_Y - BUFFER - sizeBall

    xStart2 = SCREEN_X / 2
    yStart2 = SCREEN_Y - BUFFER - sizeBall - 200

    newBall = Ball(xStart, yStart, sizeBall, (180, 180, 180), defaultMass, DEBUG_COLOR, dt)
    ball2 = Ball(xStart2, yStart2, sizeBall, BALL_COLOR, defaultMass, DEBUG_COLOR, dt)

    newBall.modifyVelocity([0, 0])
    ball2.modifyVelocity([0, 0])

    sim1.addObjectToSimulation(newBall)
    sim1.addObjectToSimulation(ball2)

    num_steps = 800 * (1/stepSize)
    curr_step = 1

    base_assertions = simulation_assertions(sim1)
    running = True
    while running and curr_step <= num_steps:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        if visualize:
            screen.fill(BACKGROUND)

        sim1.step()

        if visualize:
            sim1.draw(screen, 0)
            if show_fps:
                frame_shower.logAndDraw()
            pygame.display.update()

        curr_step += 1

    base_assertions.run_all_assertions()


def test_edge_case2(simulation) -> None:
    """
    Tests edge cases for balls bouncing on weird axises
    :param simulation:
    :return:
    """
    sizeBall = 50
    defaultMass = 1

    xStart = 500
    yStart = SCREEN_Y - BUFFER - sizeBall - 200

    xStart2 = 500
    yStart2 = SCREEN_Y - BUFFER - sizeBall

    newBall = Ball(xStart, yStart, sizeBall, (180, 180, 180), defaultMass)
    ball2 = Ball(xStart2, yStart2, sizeBall, BALL_COLOR, defaultMass)

    newBall.modifyVelocity([0, 0])
    ball2.modifyVelocity([0, 0])

    simulation.addObjectToSimulation(newBall)
    simulation.addObjectToSimulation(ball2)

    newLine = DebugLine(50, 60, (0, 0))
    newLine.setFollowObject(newBall)

    newLine = DebugLine(50, 60, (0, 0))
    newLine.setFollowObject(ball2)


def test_edge_case3(simulation) -> None:
    """
    Tests edge cases for balls bouncing on weird axises
    :param simulation:
    :return:
    """
    sizeBall = 50
    defaultMass = 1

    xStart = BUFFER + sizeBall + 150
    yStart = SCREEN_Y - BUFFER - sizeBall

    xStart2 = SCREEN_X - BUFFER - sizeBall
    yStart2 = SCREEN_Y - BUFFER - sizeBall

    newBall = Ball(xStart, yStart, sizeBall, (180, 180, 180), defaultMass)
    ball2 = Ball(xStart2, yStart2, sizeBall, BALL_COLOR, defaultMass)

    newBall.modifyVelocity([0, 0])
    ball2.modifyVelocity([-250, 0])

    simulation.addObjectToSimulation(newBall)
    simulation.addObjectToSimulation(ball2)

    newLine = DebugLine(50, 60, (0, 0))
    newLine.setFollowObject(newBall)

    newLine = DebugLine(50, 60, (0, 0))
    newLine.setFollowObject(ball2)


if __name__ == '__main__':
    print('only running this?')