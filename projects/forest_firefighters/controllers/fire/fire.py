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

from controller import Supervisor


class Tree:
    def __init__(self, node):
        self.node = node
        self.fire = None


class Fire(Supervisor):
    timeStep = 128

    def __init__(self):
        super(Fire, self).__init__()
        root = self.getRoot()
        self.children = root.getField("children")
        n = self.children.getCount()
        self.trees = []
        for i in range(n):
            node = self.children.getMFNode(i)
            if node.getTypeName() == 'Sassafras':
                self.trees.append(Tree(node))
        n = len(self.trees)
        if n == 0:
            print('No sassafras tree found.')
        else:
            print(f'Starting wildfire in a forest of {n} sassafras trees.')
            self.ignite(self.trees[0])

    def ignite(self, tree):
        translation = tree.node.getField('translation').getSFVec3f()
        tree.fire_scale = 1
        fire = f'Fire {{ translation {translation[0]} {translation[1]} {translation[2]} ' \
               f'scale {tree.fire_scale} {tree.fire_scale} {tree.fire_scale} }}'
        self.children.importMFNodeFromString(-1, fire)
        tree.fire = self.children.getMFNode(-1)
        tree.fire_translation_field = tree.fire.getField('translation')
        tree.fire_scale_field = tree.fire.getField('scale')
        tree.fire_translation = tree.fire_translation_field.getSFVec3f()
        tree.fire_count = 0

    def burn(self, tree):
        tree.fire_count += 1
        if tree.fire_count % 13 == 0:  # there are 13 images in the flame animation
            tree.fire_scale *= 1.2 if tree.fire_count < 17 * 13 else 0.8
            if tree.fire_count == 17 * 13:  # after 17 cycles of growing flames, we consider the tree is burnt
                tree.node.getField('burnt').setSFBool(True)
            tree.fire_scale_field.setSFVec3f([tree.fire_scale, tree.fire_scale, tree.fire_scale])
            if tree.fire_scale < 1:
                # tree.fire.remove()  # crashes Webots
                tree.fire = None
        t = [tree.fire_translation[0], tree.fire_translation[1], tree.fire_translation[2]]
        t[1] -= 100000 * tree.fire_scale * (tree.fire_count % 13)
        tree.fire_translation_field.setSFVec3f(t)

    def run(self):
        while True:
            if self.step(self.timeStep) == -1:
                break
            for tree in self.trees:
                if tree.fire:
                    self.burn(tree)


controller = Fire()
controller.run()
