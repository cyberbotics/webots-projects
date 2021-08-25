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

import math
import random
from controller import Supervisor, Robot

class Tree:
    def __init__(self, node):
        print(node.getTypeName())
        self.node = node
        self.fire = None
        self.fire_count = 0
        self.translation = node.getField('translation').getSFVec3f()
        self.size = node.getField('size').getSFFloat()
        print(self.translation)
        self.robustness = random.uniform(0.05, 0.15)

    def distance(self, t):  # distance with another tree
        dx = self.translation[0] - t.translation[0]
        dy = self.translation[1] - t.translation[1]
        dz = self.translation[2] - t.translation[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

class Wind():
    low_wind = 0
    high_wind = 1
    evolution_speed = 0.01

    def __init__(self):
        self.x = self.initialize_wind()
        self.y = self.initialize_wind()

    def initialize_wind(self):
        strength = random.triangular(self.low_wind, self.high_wind)
        direction = random.choice((-1, 1))
        return strength * direction

    def evolve(self):
        self.x = self.x + self.evolution_speed * random.uniform(-self.high_wind, self.high_wind)
        self.y = self.y + self.evolution_speed * random.uniform(-self.high_wind, self.high_wind)

    def corrected_distance(self, tree1, tree2, propagation_radius):
        dx = tree1.translation[0] + propagation_radius * self.x - tree2.translation[0]
        dy = tree1.translation[1] + propagation_radius * self.y - tree2.translation[1]
        dz = tree1.translation[2] - tree2.translation[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

class Fire(Supervisor):
    time_step = 128
    flame_cycle = 13  # there are 13 images in the flame animation
    flame_peak = 17   # after 17 flame cycles, the fire starts to decrease
    max_propagation = 30 # the maximum distance that the fire can propagate in meter

    def __init__(self):
        super(Fire, self).__init__()
        root = self.getRoot()
        self.children = root.getField("children")
        self.trees = []
        self.findAllTrees(root)

        self.wind = Wind()
        n = len(self.trees)
        if n == 0:
            print('No sassafras tree found.')
        else:
            print(f'Starting wildfire in a forest of {n} sassafras trees.')
            self.ignite(random.choice(self.trees))

    # find all "Sassafras" trees recursively and add them to the tree list
    def findAllTrees(self, node):
        children = node.getField("children")
        if children is not None:
            n = children.getCount()
            for i in range(n):
                child = children.getMFNode(i)
                if child.getTypeName() == "Sassafras":
                    self.trees.append(Tree(child))
                else:
                    self.findAllTrees(child)

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
        print(f'Fire strength {fire_strength}')
        for t in self.trees:
            if t == tree:
                continue
            propagation_radius = self.max_propagation * fire_strength
            distance =  self.wind.corrected_distance(tree, t, propagation_radius)

            if distance + t.robustness < propagation_radius:
                self.ignite(t)

    def run(self):
        while True:
            if self.step(self.time_step) == -1:
                break
            self.wind.evolve()
            
            print(f'wind: ({self.wind.x}, {self.wind.y})')
            for tree in self.trees:
                if tree.fire:
                    self.burn(tree)


controller = Fire()
controller.run()
