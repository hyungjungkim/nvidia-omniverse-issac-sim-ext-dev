import os
# from omni.isaac.examples.base_sample import BaseSampleExtension
from .base_sample_extension import BaseSampleExtension
from omni.isaac.hello_object_asset_converter.hello_world import HelloWorld

import omni.ui as ui
from omni.isaac.ui.ui_utils import btn_builder, str_builder

from omni.kit.window.file_importer import get_file_importer
from typing import List

import os

import asyncio
import omni.kit.asset_converter as converter
from omni.kit.asset_converter import AssetConverterContext

MENU_NAME = 'Issac Ext Dev'
EXTENSION_NAME = 'Hello Object - Asset Converter'
SUMMARY = 'This example covers converting a non-USD format of engineering 3D models such as FBX and OBJ to the USD and adding the USD asset in the Scene of the World.'

class HelloAssetConverterExt(BaseSampleExtension):
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

        self.task_ui_elements = {}
        frame = self.get_frame(index=0)
        self.build_asset_converter_ui(frame)

        self.file_name = ''
        self.file_path = ''
    
    def build_asset_converter_ui(self, frame):
        with frame:
            with ui.VStack(spacing=5):
                frame.title = 'Asset Converter'
                frame.visible = True
            
                dict = {
                        'label': 'Select Model',
                        'type': 'button',
                        'text': 'SELECT',
                        'tooltip': 'Select a non-USD model',
                        'on_clicked_fn': self._on_model_importer_button_event,
                    }
                
                self.task_ui_elements['Select Model'] = btn_builder(**dict)
                self.task_ui_elements['Select Model'].enabled = True

                dict = {
                    'label': 'Model Name',
                    'type': 'stringfield',
                    'default_val': 'None',
                    'tooltip': 'Selected model name',
                    'on_clicked_fn': None,
                    'use_folder_picker': False,
                    'read_only': True,
                }
                self.task_ui_elements['Model Name'] = str_builder(**dict)

                dict = {
                        'label': 'Convert to USD',
                        'type': 'button',
                        'text': 'CONVERT',
                        'tooltip': 'Convert the selected model to USD',
                        'on_clicked_fn': self._on_asset_converter_button_event,
                    }
                
                self.task_ui_elements['Convert to USD'] = btn_builder(**dict)
                self.task_ui_elements['Convert to USD'].enabled = False
    
    def _on_model_importer_button_event(self):
        print('Select a model')
        file_importer = get_file_importer()
        file_importer.show_window(
            title='Select a non-USD model file',
            import_button_label='Select',
            import_handler=self.import_handler
        )
    
    def import_handler(self, filename: str, dirname: str, selections: List[str] = []):
        if len(selections) > 0:
            # only converts the first file when selecting multiple files
            self.file_name = filename
            self.task_ui_elements['Model Name'].set_value(filename)
            self.file_path = selections[0]
            self.task_ui_elements['Convert to USD'].enabled = True
        else:
            self.task_ui_elements['Convert to USD'].enabled = False

    def _on_asset_converter_button_event(self):
        print('Convert to USD')
        asset_converter_context = AssetConverterContext()
        asset_converter_context.merge_all_meshes = True
        # asset_converter_context.use_meter_as_world_unit = True
        asset_converter_context.convert_fbx_to_z_up = True
        output_asset_path = os.path.splitext(self.file_path)[0]
        output_asset_path = output_asset_path + '.usd'
        # print(self.filepath)
        # print(output_asset_path)

        async def convert(input_model_path, output_asset_path):
            task_manager = converter.get_instance()
            task = task_manager.create_converter_task(input_model_path, output_asset_path,
                self.progress_callback, asset_converter_context)
            success = await task.wait_until_finished()
            if not success:
                detailed_status_code = task.get_status()
                detailed_status_error_string = task.get_error_message()
            else:
                print('Success to convert')
            
            prim_path = '/World/' + os.path.splitext(self.file_name)[0]
            # print(prim_path)
            await self._sample.add_asset_async(output_asset_path, prim_path)

        asyncio.ensure_future(convert(self.file_path, output_asset_path))

    def progress_callback(self, current_step: int, total: int):
        if current_step == total:
            print('Converting done')

    