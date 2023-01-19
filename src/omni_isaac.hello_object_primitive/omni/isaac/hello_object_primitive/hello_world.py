# from omni.isaac.examples.base_sample import BaseSample
from .base_sample import BaseSample

from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.objects import FixedCylinder
from omni.isaac.core.objects import VisualSphere

import numpy as np

class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()

    def setup_scene(self):
        print('setup_scene')
        world = self.get_world()
        world.scene.add_default_ground_plane()

        # world.scene.add(): Adds an object to the Scene
        dynamic_cuboid = world.scene.add(
            DynamicCuboid(
                prim_path='/World/dynamic_cube', name='dynamic_cube',
                position=np.array([0.0,0.0,1.0]),
                scale=np.array([5.0,5.0,5.0]),
                color=np.array([1.0,0.0,0.0]),
            )
        )
        
        fixed_cylinder = world.scene.add(
            FixedCylinder(
                prim_path='/World/fixed_cylinder', name='fixed_cylinder',
                position=np.array([-0.3,0.3,0.6]),
                scale=np.array([5.0,5.0,5.0]),
                color=np.array([0.0,1.0,0.0]),
            )
        )
        visual_sphere = world.scene.add(
            VisualSphere(
                prim_path='/World/visual_sphere', name='visual_sphere',
                position=np.array([0.3,-0.3,0.3]),
                scale=np.array([5.0,5.0,5.0]),
                color=np.array([0.0,0.0,1.0]),
            )
        )
        # world.add_physics_callback: Adds a callback which will be called before each physics step.
        world.add_physics_callback('sim_step', callback_fn=self.print_object_info)
    
    def print_object_info(self, step_size):
        world = self.get_world()
        name = 'dynamic_cube'
        object = world.scene.get_object(name)
        position, orientation = object.get_world_pose()
        linear_velocity = object.get_linear_velocity()
        print('%s\'s position : %s' % (name, str(position)))
        # print('%s\'s orientation : %s' % (name, str(orientation)))
        # print('%s\'s linear velocity : %s' % (name, str(linear_velocity)))
        
    async def setup_post_load(self):
        print('setup_post_load')

    async def setup_pre_reset(self):
        print('setup_pre_reset')

    async def setup_post_reset(self):
        print('setup_post_reset')

    def world_cleanup(self):
        print('world_cleanup')
