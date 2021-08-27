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

from controller import Supervisor, Robot

class Tree:
    robustness_variation = 2

    def __init__(self, node):
        self.node = node
        self.fire = None
        self.fire_count = 0
        self.translation = node.getField('translation').getSFVec3f()
        self.size = node.getField('size').getSFFloat()
        self.robustness = random.uniform(self.robustness_variation, self.robustness_variation)

class Wind():
    intensity_evolve = 0.01
    angle_evolve = 0.005
    random_evolution = True

    def __init__(self):
        self.intensity = random.random()
        self.angle = random.uniform(0, 2 * math.pi)

    def evolve(self):
        if self.random_evolution:
            self.intensity = max(0, min(1, self.intensity + self.intensity_evolve * random.uniform(-1, 1)))
            self.angle = self.angle + self.angle_evolve * random.uniform(-2 * math.pi, 2 * math.pi) % (2 * math.pi) 

    def update(self, message): # update the wind according to the message
        if message == "stop":
            self.random_evolution = False
        elif message == "start":
            self.random_evolution = True
        else:
            wind = json.loads(message)
            self.angle = wind["angle"]
            self.intensity = wind["intensity"]

    def correctedDistance(self, tree1, tree2, propagation_radius): # distance between two trees considering the wind
        x_wind = self.intensity * math.cos(self.angle)
        y_wind = self.intensity * math.sin(self.angle)
        dx = tree1.translation[0] + propagation_radius * x_wind - tree2.translation[0]
        dy = tree1.translation[1] + propagation_radius * y_wind - tree2.translation[1]
        dz = tree1.translation[2] - tree2.translation[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

class Fire(Supervisor):
    time_step = 128
    flame_cycle = 13  # there are 13 images in the flame animation
    flame_peak = 17   # after 17 flame cycles, the fire starts to decrease
    max_propagation = 20 # the maximum distance that the fire can propagate in meter

    def __init__(self):
        super(Fire, self).__init__()

        self.wind = Wind()

        root = self.getRoot()
        self.children = root.getField('children')

        self.trees = []

        # Add all the Sassafras trees from the forest to the list of trees
        n = self.children.getCount()
        for i in range(n):
            child = self.children.getMFNode(i)
            child_name = child.getField('name')
            if child_name is not None:
                if child_name.getSFString()  == 'uneven forest':
                    forest_children = child.getField('children')
                    if forest_children is not None:
                        m = forest_children.getCount()
                        for j in range(m):
                            forest_child = forest_children.getMFNode(j)
                            if forest_child.getTypeName() == 'Sassafras':
                                self.trees.append(Tree(forest_child))

        n = len(self.trees)
        if n == 0:
            print('No sassafras tree found.')
        else:
            print(f'Starting wildfire in a forest of {n} sassafras trees.')
            self.ignite(random.choice(self.trees))

    def ignite(self, tree):
        if tree.fire_count > 1:  # already burnt
            return
        tree.fire_scale = tree.size
        fire = f'Fire {{ translation {tree.translation[0]} {tree.translation[1]} {tree.translation[2]} ' \
               f'scale {tree.fire_scale} {tree.fire_scale} {tree.fire_scale} }}'
        self.children.importMFNodeFromString(-1, fire)
        tree.fire = self.children.getMFNode(-1)
        tree.fire_translation_field = tree.fire.getField('translation')
        tree.fire_scale_field = tree.fire.getField('scale')
        tree.fire_translation = tree.fire_translation_field.getSFVec3f()

    def burn(self, tree):
        tree.fire_count += 1
        if tree.fire_count % self.flame_cycle == 0:
            tree.fire_scale *= 1.2 if tree.fire_count < self.flame_peak * self.flame_cycle else 0.8
            if tree.fire_count == self.flame_peak * self.flame_cycle:
                tree.node.getField('burnt').setSFBool(True)
            tree.fire_scale_field.setSFVec3f([tree.fire_scale, tree.fire_scale, tree.fire_scale])
            if tree.fire_scale < 1:
                # tree.fire.remove()  # crashes Webots
                tree.fire = None
        t = [tree.fire_translation[0], tree.fire_translation[1], tree.fire_translation[2]]
        t[1] -= 100000 * tree.fire_scale * (tree.fire_count % 13)
        tree.fire_translation_field.setSFVec3f(t)
        self.propagate(tree)

    def propagate(self, tree):  # propagate fire to neighbouring trees
        fire_peak = self.flame_peak * self.flame_cycle
        fire_strength = (min(tree.fire_count, 2 * fire_peak - tree.fire_count)  / fire_peak) ** 2
        for t in self.trees:
            if t == tree:
                continue
            propagation_radius = self.max_propagation * fire_strength
            distance =  self.wind.correctedDistance(tree, t, propagation_radius)

            if distance + t.robustness < propagation_radius:
                self.ignite(t)

    def run(self):
        while True:
            if self.step(self.time_step) == -1:
                break

            message = self.wwiReceiveText()
            if message:
                self.wind.update(message)

            self.wind.evolve()
            self.wwiSendText('{"angle":%f, "intensity":%f}' % (self.wind.angle, self.wind.intensity))
            for tree in self.trees:
                if tree.fire:
                    self.burn(tree)


controller = Fire()
controller.run()
