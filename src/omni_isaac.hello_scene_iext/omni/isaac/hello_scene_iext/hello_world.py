from omni.isaac.core import World
from omni.isaac.core.scenes.scene import Scene
from omni.isaac.core.utils.stage import create_new_stage_async

class HelloWorld(object):
    def __init__(self) -> None:
        self._world = None
    
    def get_world(self):
        return self._world

    def setup_scene(self):
        print('setup a scene')
        world = self.get_world()
        world.scene.add_default_ground_plane()

    async def load_world_async(self):
        if World.instance() is None:
            await create_new_stage_async()
            self._world = World()
            # await self._world.initialize_simulation_context_async()
            self.setup_scene()
        else:
            self._world = World.instance()
        await self._world.reset_async() # Resets the stage to its initial state and each object included in the Scene to its default state
        await self._world.pause_async() # Pauses the physics simulation