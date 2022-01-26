# Import the necessary packages
from cursesmenu import *
from cursesmenu.items import *

# Create the menu
menu = CursesMenu("Title", "Subtitle")

# Create some items

# MenuItem is the base class for all items, it doesn't do anything when selected
menu_item = MenuItem("Menu Item")

# A FunctionItem runs a Python function when selected

def test():
    return
function_item = FunctionItem("Call a Python function", test,)

# A CommandItem runs a console command
command_item = CommandItem("Run a console command", 'rostopic pub /state_request std_msgs/UInt8 "data: 0" -1')

# A SelectionMenu constructs a menu from a list of strings
selection_menu = SelectionMenu(["item1", "item2", "item3"])

# A SubmenuItem lets you add a menu (the selection_menu above, for example)
# as a submenu of another menu
submenu_item = SubmenuItem("Submenu item", selection_menu, menu)

# Once we're done creating them, we just add the items to the menu
menu.items.append(menu_item)
menu.items.append(function_item)
menu.items.append(command_item)
menu.items.append(submenu_item)

# Finally, we call show to show the menu and allow the user to interact
menu.show()