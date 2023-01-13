# from omni.isaac.examples.base_sample import BaseSample
from .base_sample import BaseSample

class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()

    def setup_scene(self):
        print('setup_scene')
        world = self.get_world()
        world.scene.add_default_ground_plane()

    async def setup_post_load(self):
        print('setup_post_load')

    async def setup_pre_reset(self):
        print('setup_pre_reset')

    async def setup_post_reset(self):
        print('setup_post_reset')

    def world_cleanup(self):
        print('world_cleanup')
