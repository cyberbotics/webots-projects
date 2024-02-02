# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
This supervisor controller simulate a wild fire in a Sassafras forest.
"""

import json
import math
import random

from controller import Supervisor

def rotate(matrix, vector):
    result = []
    for i in range(3):
        r = 0
        for j in range(3):
            r += matrix[3 * i + j] * vector[j]
        result.append(r)
    return result


class Tree:
    ROBUSTNESS_VARIATION = 2

    def __init__(self, node):
        self.node = node
        self.fire = None
        self.smoke = None
        self.fire_count = 0
        self.translation = node.getField('translation').getSFVec3f()
        self.scale = node.getField('scale').getSFFloat()
        self.robustness = random.uniform(self.ROBUSTNESS_VARIATION, self.ROBUSTNESS_VARIATION)
        node.getField('burnt').setSFBool(False)  # Ensure normal tree as initial state

    def stopFire(self):
        if self.fire:
            fire_translation_field = self.fire.getField('translation')
            fire_translation = fire_translation_field.getSFVec3f()
            t = [fire_translation[0], fire_translation[1], fire_translation[2]]
            # t[2] = 1000000
            fire_translation_field.setSFVec3f(t)
            self.fire = None
        if self.smoke:
            smoke_translation_field = self.smoke.getField('translation')
            smoke_translation = smoke_translation_field.getSFVec3f()
            t = [smoke_translation[0], smoke_translation[1], smoke_translation[2]]
            t[2] = 1000000
            smoke_translation_field.setSFVec3f(t)
            self.smoke = None

    def distance(self, coordinates):
        dx = self.translation[0] - coordinates[0]
        dy = self.translation[1] - coordinates[1]
        dz = self.translation[2] - coordinates[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)


class Robot():
    MAX_WATER_RADIUS = 0.3

    def __init__(self, node):
        self.node = node
        self.name = node.getField('name').getSFString()
        self.type = node.getTypeName()
        self.droppingWater = False
        self.waterBalls = []

    def dropWater(self, children, quantity):
        position = self.node.getField('translation').getSFVec3f()
        radius = min(self.MAX_WATER_RADIUS, 0.01 * quantity)
        water = f'Water {{ translation {position[0]} {position[1]} {position[2]} ' \
                f'radius {radius} ' \
                f'name "water {len(self.waterBalls)} {self.name}" }}'
        children.importMFNodeFromString(-1, water)
        waterNode = children.getMFNode(-1)
        self.waterBalls.append(waterNode)

        # if this is a "Spot" robot, the water it throws will have an initial velocity
        if self.type == "Spot":
            rotationMatrix = self.node.getOrientation()
            orientation = rotate(rotationMatrix, [0, 0, -1])
            velocity = [i * 10 for i in orientation] + [0, 0, 0]
            waterNode.setVelocity(velocity)

    def cleanWater(self):
        for waterBall in self.waterBalls:
            altitude = waterBall.getField('translation').getSFVec3f()[2]
            if altitude < 0:
                self.waterBalls.remove(waterBall)
                waterBall.remove()

    def altitude(self):
        return self.node.getField('translation').getSFVec3f()[2]

class Wind():
    INTENSITY_EVOLVE = 0.01
    ANGLE_EVOLVE = 0.005
    RANDOM_EVOLUTION = True

    def __init__(self):
        self.intensity = random.random()
        self.angle = random.uniform(0, 2 * math.pi)
        print('Wind angle', self.angle)
        print('Wind intensity', self.intensity)

    def evolve(self):
        if self.RANDOM_EVOLUTION:
            self.intensity = max(0, min(1, self.intensity + self.INTENSITY_EVOLVE * random.uniform(-1, 1)))
            self.angle = self.angle + self.ANGLE_EVOLVE * random.uniform(-2 * math.pi, 2 * math.pi) % (2 * math.pi)

    def update(self, message):   # update the wind according to the message
        if message == "stop":
            self.RANDOM_EVOLUTION = False
        elif message == "start":
            self.RANDOM_EVOLUTION = True
        else:
            wind = json.loads(message)
            self.angle = wind["angle"]
            self.intensity = wind["intensity"]

    def correctedDistance(self, tree1, tree2, propagation_radius):  # distance between two trees considering the wind
        x_wind = self.intensity * math.cos(self.angle)
        y_wind = self.intensity * math.sin(self.angle)
        dx = tree1.translation[0] + propagation_radius * x_wind - tree2.translation[0]
        dy = tree1.translation[1] - tree2.translation[1]
        dz = tree1.translation[2] + propagation_radius * y_wind - tree2.translation[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

def fire_shape_url(shape_node):
    if shape_node.getBaseTypeName() == 'Shape':
        appr_node = shape_node.getField('appearance').getSFNode()
        return appr_node.getField('url').getMFString(0)
    else:
        print('Node is not an instance of Shape, nothing to return')
        return None

class Fire(Supervisor):
    FLAME_CYCLE = 13        # there are 13 images in the flame animation
    FLAME_PEAK = 17         # after 13 flame cycles, the fire starts to decrease
    MAX_PROPAGATION = 5    # the maximum distance that the fire can propagate in meter
    MAX_EXTINCTION = 4      # the maximum distance from a tree where water can stop its fire in meter
    FIRE_DURATION = 50

    def __init__(self):
        super(Fire, self).__init__()

        self.time_step = int(self.getBasicTimeStep())
        self.fire_clock = 0

        self.wind = Wind()

        root = self.getRoot()
        self.children = root.getField('children')

        self.trees = []

        self.robots = []
        self.fire_sprites = []

        n = self.children.getCount()
        for i in range(n):
            child = self.children.getMFNode(i)

            # Add all the Sassafras trees from the forest to the list of trees
            child_name = child.getField('name')
            if child_name is not None:
                if child_name.getSFString() == 'uneven forest':
                    forest_children = child.getField('children')
                    if forest_children is not None:
                        m = forest_children.getCount()
                        for j in range(m):
                            forest_child = forest_children.getMFNode(j)
                            if forest_child.getTypeName() == 'Sassafras':
                                self.trees.append(Tree(forest_child))

            # Add all the robot that may extinguish the fire
            if child.getBaseTypeName() == 'Robot':
                if child.getField('translation') is not None and child.getField('customData') is not None:
                    self.robots.append(Robot(child))

        n = len(self.trees)
        if n == 0:
            print('No sassafras tree found.')
        else:
            print(f'Starting wildfire in a forest of {n} sassafras trees.')
            self.ignite(random.choice(self.trees))

    def ignite(self, tree):
        if tree.fire_count > 1:  # already burnt
            return
        tree.fire_scale = tree.scale
        fire = f'Fire {{ translation {tree.translation[0]} {tree.translation[1]} {tree.translation[2]} ' \
               f'scale {tree.fire_scale} {tree.fire_scale} {tree.fire_scale} }}'
        self.children.importMFNodeFromString(-1, fire)
        tree.fire = self.children.getMFNode(-1)
        tree.fire_translation_field = tree.fire.getField('translation')
        tree.fire_scale_field = tree.fire.getField('scale')
        tree.fire_translation = tree.fire_translation_field.getSFVec3f()

        node_child = tree.fire.getProtoField('children').getMFNode(0)
        if node_child.getBaseTypeName() == 'Shape':
            appr_node = node_child.getField('appearance').getSFNode()
            tree.fire_shape_field = appr_node.getField('url')

        if len(self.fire_sprites) == 0:
            self.fire_sprites.append(fire_shape_url(node_child))
            for i in range(self.FLAME_CYCLE):
                node_child = tree.fire.getProtoField('children').getMFNode(i)
                if node_child.getBaseTypeName() == 'Pose':
                    txr_node = node_child.getField('children').getMFNode(0)
                    self.fire_sprites.append(fire_shape_url(txr_node))

    def burn(self, tree):
        if self.update_fire:
            tree.fire_count += 1
            if tree.fire_count % self.FLAME_CYCLE == 0:
                tree.fire_scale *= 1.2 if tree.fire_count < self.FLAME_PEAK * self.FLAME_CYCLE else 0.8
                if tree.fire_count == self.FLAME_PEAK * self.FLAME_CYCLE:
                    tree.node.getField('burnt').setSFBool(True)
                tree.fire_scale_field.setSFVec3f([tree.fire_scale, tree.fire_scale, tree.fire_scale])

                if tree.fire_scale < tree.scale:
                    tree.stopFire()
            t = [tree.fire_translation[0], tree.fire_translation[1], tree.fire_translation[2]]
            tree.fire_translation_field.setSFVec3f(t)
            if tree.fire:
                sprite_field = tree.fire.getField('spriteBase')
                sprite_field.setMFString(0, '../protos/' + self.fire_sprites[tree.fire_count % self.FLAME_CYCLE])
            self.propagate(tree)
            if tree.fire_count == 1:
                smoke = f'Smoke {{ translation {tree.translation[0]} {tree.translation[1]} {tree.translation[2]} ' \
                    f'scale {0.01} {0.01} {0.01} }}'
                self.children.importMFNodeFromString(-1, smoke)
                tree.smoke = self.children.getMFNode(-1)
                tree.smoke_translation_field = tree.smoke.getField('translation')
                tree.smoke_scale_field = tree.smoke.getField('scale')
                tree.smoke_translation = tree.smoke_translation_field.getSFVec3f()
            if 0 < tree.fire_count < 70:
                tree.smoke_scale_field.setSFVec3f([tree.fire_count/100, tree.fire_count/100, tree.fire_count/100])

    def propagate(self, tree):  # propagate fire to neighbouring trees
        fire_peak = self.FLAME_PEAK * self.FLAME_CYCLE
        fire_strength = (min(tree.fire_count, 2 * fire_peak - tree.fire_count) / fire_peak) ** 2
        for t in self.trees:
            if t == tree:
                continue
            propagation_radius = self.MAX_PROPAGATION * fire_strength
            distance = self.wind.correctedDistance(tree, t, propagation_radius)

            if distance + t.robustness < propagation_radius * math.sqrt(tree.scale):
                self.ignite(t)

    def checkExtinction(self, tree):  # check and extinct the fire if there is water close enough
        for robot in self.robots:
            for water in robot.waterBalls:
                water_position = water.getField('translation').getSFVec3f()
                water_radius = water.getField('radius').getSFFloat()
                water_extinction_radius = self.MAX_EXTINCTION * water_radius / robot.MAX_WATER_RADIUS
                if tree.distance(water_position) < water_extinction_radius:
                    fire_size = tree.scale * tree.fire_scale / 20
                    if water_radius / robot.MAX_WATER_RADIUS > fire_size:
                        tree.stopFire()
                        return True
        return False

    def run(self):
        start_fire_now = False
        while True:
            step = self.step(self.time_step)
            if step == -1:
                break

            message = self.wwiReceiveText()
            if message:
                print('message', message)
                self.wind.update(message)
            self.wind.evolve()

            self.wwiSendText('{"angle":%f, "intensity":%f}' % (self.wind.angle, self.wind.intensity))

            for robot in self.robots:
                robot.cleanWater()

                customData = robot.node.getField('customData').getSFString()
                if customData != "":
                    quantity_of_water = int(customData)
                    if quantity_of_water > 0:
                        robot.dropWater(self.children, quantity_of_water)
            if start_fire_now:
                # update the fire_clock
                if self.fire_clock == self.FIRE_DURATION:
                    self.update_fire = True
                    self.fire_clock = 0
                else:
                    self.update_fire = False
                    self.fire_clock += 1

                extinction = []
                for tree in self.trees:
                    if tree.fire:
                        self.burn(tree)
                        extinction.append(self.checkExtinction(tree))

                if True in extinction:
                        self.ignite(random.choice(self.trees))
            else:
                for robot in self.robots: # the simulation starts when the mavic got an altitude > 40
                    if not start_fire_now and robot.name == "Mavic 2 PRO" and robot.altitude() > 40:
                            start_fire_now = True
                print('waiting altitude', start_fire_now)

controller = Fire()
controller.run()
