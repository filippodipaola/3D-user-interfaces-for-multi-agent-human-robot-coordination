using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using RosSharp.RosBridgeClient;

public class OnRightClick : MonoBehaviour {

    private bool rightClickAndSelected = false;
    private float mouse_offset = 10f;
    private Camera camera;
    private float mousex;
    private float mousey;
    private Vector3 mousePos;
    public bool clicked_off = false;

    private void Start()
    {
        camera = Camera.main;
    }

    /*private void OnMouseOver()
    {
        if (Input.GetMouseButtonDown(1))
        {
            if (selectAgent.currentlySelected.Count != 0)
            {
                rightClickAndSelected = true;
                mousex = Input.mousePosition.x;
                mousey = Input.mousePosition.y;
                mousePos = Input.mousePosition;
                print("Agent has been selected, and right click has been pressed.");
            }
            else
            {
                rightClickAndSelected = false;
                print("Right cick has been clicked, but NO AGENT SELECTED!");
            }
        }
    }*/

    /*private void OnGUI()
    {
        if (rightClickAndSelected)
        {
            GUI.Box(new Rect(mousex + mouse_offset, Screen.height - mousey - mouse_offset, 200, 50), "");
            if (GUI.Button(new Rect(mousex + mouse_offset, Screen.height - mousey - mouse_offset, 200, 50), "Move Here"))
            {
                print("Button has been clicked!");
                RaycastHit hit;
                Ray ray = camera.ScreenPointToRay(mousePos);
                if (Physics.Raycast(ray, out hit))
                {
                    Vector3 objectPos = hit.point;
                    print("Raycast Hit an Object at: " + objectPos);
                    foreach (selectAgent s in selectAgent.currentlySelected)
                    {
                        GameWorldSubscriber.sendUserInput("MOVE|" + s.agentName + "|"+objectPos.x+","+objectPos.z);
                        rightClickAndSelected = false;
                    }
                    selectAgent.DeselectAll();
                }
            }
        }
    }*/

    private void OnMouseDown()
    {
        rightClickAndSelected = false;
        clicked_off = true;
    }
}
