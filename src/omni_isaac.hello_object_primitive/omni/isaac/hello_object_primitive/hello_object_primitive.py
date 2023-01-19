import os
# from omni.isaac.examples.base_sample import BaseSampleExtension
from .base_sample_extension import BaseSampleExtension
from omni.isaac.hello_object_primitive.hello_world import HelloWorld

MENU_NAME = 'Issac Ext Dev'
EXTENSION_NAME = 'Hello Object - Primitive'
SUMMARY = 'This example contains how to add primitive Omniverse geometric objects in the Scene of the World.'

class HelloObjectExt(BaseSampleExtension):
    def on_startup(self, ext_id: str):
        super().on_startup(ext_id)
        print('on_startup')
        super().start_extension(
            menu_name='',
            submenu_name='',
            name=EXTENSION_NAME,
            title=EXTENSION_NAME,
            doc_link='',
            overview=SUMMARY,
            file_path=os.path.abspath(__file__),
            sample=HelloWorld(),
            keep_window_open=True,
        )