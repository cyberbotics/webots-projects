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
        self.fire_level = 0
        self.fire_loop = 0


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
                print('Added one tree')
            print(node.getTypeName())
        self.ignite(self.trees[0])

    def ignite(self, tree):
        self.children.importMFNodeFromString(-1, 'Fire { translation 0 4 0 scale 10 10 10 }')

    def run(self):
        while True:
            if self.step(self.timeStep) == -1:
                break


controller = Fire()
controller.run()
