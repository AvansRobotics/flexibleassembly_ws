#!/usr/bin/env python

# license removed for brevity
import rospy
from enum import IntEnum
from cursesmenu import *
from cursesmenu.items import *
from std_msgs.msg import UInt8
import sys
import time


class States(IntEnum):
  IDLE = 0
  INITIALIZE = 1
  READ_CONFIG = 2
  PICK_ARDUINO = 3
  REQUEST_COMPONENT = 4
  REQUEST_PLACE_POSE = 5
  ROTATE_ARDUINO = 6
  PICK_COMPONENT = 7
  PLACE_COMPONENT = 8
  PLACE_ARDUINO= 9
  DEINITIALIZE = 10
  SHUTDOWN = 11
  ERROR = 12
  ASSEMBLE = 13


currentState = None
pub_state_request = None


def callback(data):
    global currentState
    currentState = data.data
    showMenu()

def showMenu(pub_state_request):
    # Create the menu
    global States
    menu = CursesMenu("Current state = " + str(currentState), "Menu:")
    menu.highlight = 1

    # Create some items

    # MenuItem is the base class for all items, it doesn't do anything when selected
    # menu_item = MenuItem("Menu Item")
    pub_state_request.publish(0)
    # A FunctionItem runs a Python function when selected
    #function_item_idle = FunctionItem("Idle", idle(), ["Enter an input"])
    function_item_initialize = FunctionItem("Initialize", pub_state_request.publish, [States.INITIALIZE.value])
    function_item_read_config = FunctionItem("Read config", pub_state_request.publish, [States.READ_CONFIG.value])
    function_item_pick_arduino = FunctionItem("Pick Arduino", pub_state_request.publish, [States.READ_CONFIG.value])
    function_item_request_component = FunctionItem("Request component", pub_state_request.publish, [States.REQUEST_COMPONENT.value])
    function_item_request_place_pose = FunctionItem("Request place pose", pub_state_request.publish, [States.REQUEST_PLACE_POSE.value])
    function_item_rotate_arduino = FunctionItem("Rotate Arduino", pub_state_request.publish, [States.ROTATE_ARDUINO.value])
    function_item_pick_component = FunctionItem("Pick component", pub_state_request.publish, [States.PICK_COMPONENT.value])
    function_item_place_component = FunctionItem("Place component", pub_state_request.publish, [States.PLACE_COMPONENT.value])
    function_item_place_arduino = FunctionItem("PlaceArduino", pub_state_request.publish, [States.PLACE_ARDUINO.value])
    function_item_deinitialize = FunctionItem("De-initialize", pub_state_request.publish, [States.DEINITIALIZE.value])
    function_item_shutdown = FunctionItem("Shutdown nodes", pub_state_request.publish, [States.SHUTDOWN.value])
    function_item_assemble = FunctionItem("Start the assemble process (config needs to be read)", pub_state_request.publish, [States.ASSEMBLE.value])

    # A CommandItem runs a console command
    command_item = CommandItem("Run a console command", "touch hello.txt")

    # A SelectionMenu constructs a menu from a list of strings
    selection_menu = SelectionMenu(["item1", "item2", "item3"])

    # A SubmenuItem lets you add a menu (the selection_menu above, for example)
    # as a submenu of another menu
    submenu_item = SubmenuItem("Submenu item", selection_menu, menu)

    # Once we're done creating them, we just add the items to the menu
    #menu.items.append(MenuItem("Initialize"))
    
    menu.items.append(function_item_initialize)
    menu.items.append(function_item_read_config)
    menu.items.append(function_item_pick_arduino)
    menu.items.append(function_item_request_component)
    menu.items.append(function_item_request_place_pose)
    menu.items.append(function_item_rotate_arduino)
    menu.items.append(function_item_pick_component)
    menu.items.append(function_item_place_component)
    menu.items.append(function_item_place_arduino)
    menu.items.append(function_item_deinitialize)
    menu.items.append(function_item_shutdown)
    menu.items.append(function_item_assemble)
    #menu.items.append(command_item)
    #menu.items.append(submenu_item)

    # Finally, we call show to show the menu and allow the user to interact
    
    menu.show()
    

if __name__ == '__main__':
    rospy.init_node('flexible_assembly_ui', anonymous=True, disable_signals=True)
    rate = rospy.Rate(10) # 10hz

    pub_state_request = rospy.Publisher('/state_request', UInt8, queue_size=1)
    rospy.Subscriber("/current_state", UInt8, callback)
    
    

    showMenu(pub_state_request)

