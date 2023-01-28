# from omni.isaac.examples.base_sample import BaseSample
from .base_sample import BaseSample

from omni.isaac.core.utils.stage import add_reference_to_stage

class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()

    def setup_scene(self):
        print('setup_scene')
        world = self.get_world()
        world.scene.add_default_ground_plane()
    
    async def add_asset_async(self, asset_path, asset_prim):
        world = self.get_world()
        add_reference_to_stage(asset_path, asset_prim)

    async def setup_post_load(self):
        print('setup_post_load')

    async def setup_pre_reset(self):
        print('setup_pre_reset')

    async def setup_post_reset(self):
        print('setup_post_reset')

    def world_cleanup(self):
        print('world_cleanup')
