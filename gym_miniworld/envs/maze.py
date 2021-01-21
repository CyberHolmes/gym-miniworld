import numpy as np
import math
from gym import spaces
from ..miniworld import MiniWorldEnv, Room
from ..entity import Box, ImageFrame
from ..params import DEFAULT_PARAMS
from ..math import *

class Maze(MiniWorldEnv):
    """
    Maze environment in which the agent has to reach a red box
    """

    def __init__(
        self,
        num_rows=8,
        num_cols=8,
        room_size=3,
        max_episode_steps=None,
        **kwargs
    ):
        self.num_rows = num_rows
        self.num_cols = num_cols
        self.room_size = room_size
        self.gap_size = 0.25

        super().__init__(
            max_episode_steps = max_episode_steps or num_rows * num_cols * 24,
            **kwargs
        )

        # Allow only the movement actions
        self.action_space = spaces.Discrete(self.actions.move_forward+1)

    def _gen_world(self):
        rows = []

        # For each row
        for j in range(self.num_rows):
            row = []

            # For each column
            for i in range(self.num_cols):

                min_x = i * (self.room_size + self.gap_size)
                max_x = min_x + self.room_size

                min_z = j * (self.room_size + self.gap_size)
                max_z = min_z + self.room_size

                room = self.add_rect_room(
                    min_x=min_x,
                    max_x=max_x,
                    min_z=min_z,
                    max_z=max_z,
                    wall_tex='brick_wall',
                    #floor_tex='asphalt'
                )
                row.append(room)

            rows.append(row)

        visited = set()

        def visit(i, j):
            """
            Recursive backtracking maze construction algorithm
            https://stackoverflow.com/questions/38502
            """

            room = rows[j][i]

            visited.add(room)

            # Reorder the neighbors to visit in a random order
            neighbors = self.rand.subset([(0,1), (0,-1), (-1,0), (1,0)], 4)

            # For each possible neighbor
            for dj, di in neighbors:
                ni = i + di
                nj = j + dj

                if nj < 0 or nj >= self.num_rows:
                    continue
                if ni < 0 or ni >= self.num_cols:
                    continue

                neighbor = rows[nj][ni]

                if neighbor in visited:
                    continue

                if di == 0:
                    self.connect_rooms(room, neighbor, min_x=room.min_x, max_x=room.max_x)
                elif dj == 0:
                    self.connect_rooms(room, neighbor, min_z=room.min_z, max_z=room.max_z)

                visit(ni, nj)

        # Generate the maze starting from the top-left corner
        visit(0, 0)

        self.box = self.place_entity(Box(color='red'))

        self.place_agent()

    def step(self, action):
        obs, reward, done, info = super().step(action)   
        #print(intersect_circle_segs(self.agent.pos, np.linalg.norm(self.box.pos - self.agent.pos), self.wall_segs))
        #print(not intersect_wall_points(self.agent.pos,self.box.pos,self.wall_segs))
        # print(self.wall_segs.shape)
        # print(self.wall_segs[0])
        # qx1, _, qz1 = self.wall_segs[0,1,:]
        # print(qx1)
        # print(qz1)
        # if not intersect_wall_points(self.agent.pos,self.box.pos,self.wall_segs):
        #     #print(np.linalg.norm(self.box.pos - self.agent.pos)/10000)
        #     reward += 0.001-np.linalg.norm(self.box.pos - self.agent.pos)/10000
        #reward -= np.linalg.norm(self.box.pos - self.agent.pos)/10000
        #print(np.linalg.norm(self.box.pos - self.agent.pos)/10000)

        # for i in range(len(self.agent.past_pos)-3):
        #     dist = np.linalg.norm(self.agent.pos- self.agent.past_pos[i])
        #     if dist < 1.5 * self.max_forward_step:
        #         reward -= 0.000001
        #         print(reward)
        
        # self.agent.past_pos=np.append(self.agent.past_pos,[self.agent.pos],axis=0)
        # if len(self.agent.past_pos)>20: #only keep the last 20 steps
        #     self.agent.past_pos=np.delete(self.agent.past_pos,0,0)
        #print(np.linalg.norm(self.box.pos - self.agent.pos))
        # if not intersect_wall_points(self.agent.pos,self.box.pos,self.wall_segs):
        #     reward -= np.linalg.norm(self.box.pos - self.agent.pos)/100
        # else:
        #     reward -= 0.1 
        
        #reward -= np.linalg.norm(self.box.pos - self.agent.pos)/1000

        if self.near(self.box):
            reward += self._reward()
            done = True

        return obs, reward, done, info

class MazeS2(Maze):
    def __init__(self):
        super().__init__(num_rows=2, num_cols=2)

class MazeS3(Maze):
    def __init__(self):
        super().__init__(num_rows=3, num_cols=3)

class MazeS3Fast(Maze):
    def __init__(self, forward_step=0.7, turn_step=45):

        # Parameters for larger movement steps, fast stepping
        params = DEFAULT_PARAMS.no_random()
        params.set('forward_step', forward_step)
        params.set('turn_step', turn_step)

        max_steps = 300

        super().__init__(
            num_rows=3,
            num_cols=3,
            params=params,
            max_episode_steps=max_steps,
            domain_rand=False
        )
