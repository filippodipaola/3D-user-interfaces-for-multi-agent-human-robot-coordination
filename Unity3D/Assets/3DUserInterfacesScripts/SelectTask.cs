using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;

public class SelectTask : MonoBehaviour {

    public static SelectTask selectedTask = new SelectTask();
    public bool rightClickAndSelected = false;
    private float mouse_offset = 10f;
    private Camera camera;
    private float mousex;
    private float mousey;
    private Vector3 mousePos;
    public string taskName;
    public string taskType;

    private void Start()
    {
        camera = Camera.main;
    }

    private bool checkIfCompatible(selectAgent a)
    {
        if (taskType == "Construction")
        {
            return (a.agentType == "ConstructionAgent");
        } else
        {
            return (a.agentType == "TransportAgent");
        }
        
    }

    private void OnMouseOver()
    {
        bool is_compatible = false;
        if (Input.GetMouseButtonDown(1))
        {
            if (selectAgent.currentlySelected.Count != 0)
            {
                foreach (selectAgent s in selectAgent.currentlySelected)
                {
                    is_compatible = checkIfCompatible(s);
                }
                if (is_compatible)
                {
                    selectedTask.rightClickAndSelected = false;
                    selectedTask = this;
                    rightClickAndSelected = true;
                    mousex = Input.mousePosition.x;
                    mousey = Input.mousePosition.y;
                    mousePos = Input.mousePosition;
                    print("Agent has been selected, and right click has been pressed.");
                } else
                {
                    rightClickAndSelected = false;
                    print("Right click has been clicked, but INCORRECT AGENT SELECTED!");
                }
                
            }
            else
            {
                rightClickAndSelected = false;
                print("Right cick has been clicked, but NO AGENT SELECTED!");
            }
        }
    }

    private void OnGUI()
    {
        if (rightClickAndSelected)
        {
            GUI.Box(new Rect(mousex + mouse_offset, Screen.height - mousey - mouse_offset, 200, 50), "");
            if (GUI.Button(new Rect(mousex + mouse_offset, Screen.height - mousey - mouse_offset, 200, 50), "Assigned Agent to Task"))
            {
                foreach (selectAgent s in selectAgent.currentlySelected)
                {
                    print("Button has been clicked!");
                    GameWorldSubscriber.sendUserInput("ASSIGN|" + s.agentName + "|" + taskName);
                }

                rightClickAndSelected = false;


            }
        }
    }

    private void OnMouseDown()
    {
        rightClickAndSelected = false;
    }
}

