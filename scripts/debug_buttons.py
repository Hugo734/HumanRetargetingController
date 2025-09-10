#!/usr/bin/env python3

import openvr
import time

def debug_buttons():
    """Simple script to debug which button is which on Valve Index Controllers"""
    
    # Initialize OpenVR
    vr_system = openvr.init(openvr.VRApplication_Other)
    
    print("Button Debug Script Started!")
    print("Press any button on your controllers to see which one it is...")
    print("Press Ctrl+C to exit")
    print("-" * 50)
    
    # Button name mapping
    button_names = {
        openvr.k_EButton_System: "System",
        openvr.k_EButton_ApplicationMenu: "Application Menu",
        openvr.k_EButton_Grip: "Grip",
        openvr.k_EButton_DPad_Left: "DPad Left",
        openvr.k_EButton_DPad_Up: "DPad Up", 
        openvr.k_EButton_DPad_Right: "DPad Right",
        openvr.k_EButton_DPad_Down: "DPad Down",
        openvr.k_EButton_A: "A Button",
        openvr.k_EButton_ProximitySensor: "Proximity Sensor",
        openvr.k_EButton_Axis0: "Joystick X",
        openvr.k_EButton_Axis1: "Joystick Y", 
        openvr.k_EButton_Axis2: "Trigger",
        openvr.k_EButton_Axis3: "Grip Analog",
        openvr.k_EButton_Axis4: "Joystick Click",
        openvr.k_EButton_SteamVR_Touchpad: "Touchpad",
        openvr.k_EButton_SteamVR_Trigger: "Trigger",
    }
    
    # Track previous button states to detect presses
    prev_button_states = {}
    
    try:
        while True:
            # Get device data
            device_data_list = vr_system.getDeviceToAbsoluteTrackingPose(
                openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount
            )
            
            for device_idx in range(openvr.k_unMaxTrackedDeviceCount):
                if not device_data_list[device_idx].bDeviceIsConnected:
                    continue
                    
                device_type = vr_system.getTrackedDeviceClass(device_idx)
                device_sn = vr_system.getStringTrackedDeviceProperty(device_idx, openvr.Prop_SerialNumber_String)
                
                # Only process controllers
                if device_type == openvr.TrackedDeviceClass_Controller:
                    # Get controller state
                    controller_state = vr_system.getControllerState(device_idx)[1]
                    
                    # Check each button
                    for button_id, button_name in button_names.items():
                        # Check if button is pressed
                        is_pressed = bool(controller_state.ulButtonPressed & (1 << button_id))
                        is_touched = bool(controller_state.ulButtonTouched & (1 << button_id))
                        
                        # Create unique key for this device and button
                        key = f"{device_sn}_{button_id}"
                        
                        # Detect button press (wasn't pressed before, now is pressed)
                        if is_pressed and (key not in prev_button_states or not prev_button_states[key]):
                            print(f"🎮 {device_sn}: {button_name} PRESSED!")
                            
                        # Detect button release (was pressed before, now not pressed)
                        elif not is_pressed and (key in prev_button_states and prev_button_states[key]):
                            print(f"🎮 {device_sn}: {button_name} RELEASED!")
                            
                        # Update previous state
                        prev_button_states[key] = is_pressed
                    
                    # Also show axis values for analog inputs
                    if controller_state.rAxis[0].x != 0 or controller_state.rAxis[0].y != 0:
                        print(f"🎮 {device_sn}: Joystick X={controller_state.rAxis[0].x:.2f}, Y={controller_state.rAxis[0].y:.2f}")
                    
                    if controller_state.rAxis[1].x != 0:
                        print(f"🎮 {device_sn}: Trigger={controller_state.rAxis[1].x:.2f}")
                        
                    if controller_state.rAxis[2].x != 0:
                        print(f"🎮 {device_sn}: Grip Analog={controller_state.rAxis[2].x:.2f}")
            
            time.sleep(0.1)  # 10Hz update rate
            
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        openvr.shutdown()

if __name__ == "__main__":
    debug_buttons()


