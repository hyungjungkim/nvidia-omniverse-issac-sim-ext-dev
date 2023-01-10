# for extension
import omni.ext
import omni.appwindow # for setup_ui_headers
import omni.ui as ui
from omni.isaac.ui.ui_utils import *
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription
import weakref # for menu callback event
import gc

EXTENSION_NAME = 'Hello Extension'
MENU_NAME = 'Issac Ext Dev'

class HelloExt(omni.ext.IExt):
    def on_startup(self, ext_id):
        print(EXTENSION_NAME + ' startup')
        self._ext_id = ext_id
        self._menu_items = None
        self._windows = None
        self._ui_components = {}

        self.add_menu()

        self._build_ui()
    
    def on_shutdown(self):
        print(EXTENSION_NAME + ' shutdown')
        remove_menu_items(self._menu_items, MENU_NAME)
        self._menu_items = None
        self._window = None
        self._ui_components = {}
        gc.collect()
    
    def add_menu(self):
        menu_item = [MenuItemDescription(name=EXTENSION_NAME, onclick_fn=lambda a=weakref.proxy(self): a._menu_callback())]
        self._menu_items = menu_item
        # self._menu_items = [MenuItemDescription(name='Test', sub_menu=menu_item)]
        add_menu_items(self._menu_items, MENU_NAME)

    def _menu_callback(self):
        self._build_ui()

    def _build_ui(self):
        if not self._windows:
            self._window = ui.Window(EXTENSION_NAME, width=300, height=300, 
            dockPreference=ui.DockPreference.LEFT_BOTTOM)
        
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                title = EXTENSION_NAME
                overview = ('This a hello extension for developing an Isaac Sim application.')
                setup_ui_headers(self._ext_id, __file__, title=title, overview=overview)

                self.build_test_gui_grid()
        
        print(EXTENSION_NAME + ' build ui')

    def build_test_gui_grid(self):
        ui_components = {
            'button-1': {
                'label': 'Button 1',
                'type': 'button',
                'text': 'Test 1',
                'tooltip': 'Click for testing',
                'on_clicked_fn': self._on_button_1_clicked,
            },
            'button-2': {
                'label': 'Button 2',
                'type': 'button',
                'text': 'Test 2',
                'tooltip': 'Click for testing',
                'on_clicked_fn': self._on_button_2_clicked,
            },
            'button-3': {
                'label': 'Button 3',
                'type': 'button',
                'text': 'Test 3',
                'tooltip': 'Click for testing',
                'on_clicked_fn': self._on_button_3_clicked,
            },
        }

        self._grid = ui.CollapsableFrame(
            title="Sim control",
            height=0,
            collapsed=False,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        with self._grid:
            with ui.VStack(spacing=5, height=0):
                for key, value in ui_components.items():
                    # Button
                    if value["type"] == "button":
                        self._ui_components["btn_" + value["label"]] = btn_builder(**value)
    
    def _on_button_1_clicked(self):
        print('Button 1 clicked')
    
    def _on_button_2_clicked(self):
        print('Button 2 clicked')

    def _on_button_3_clicked(self):
        print('Button 3 clicked')	