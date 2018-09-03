using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;

public class SelectTask_vr : MonoBehaviour {

    public static SelectTask_vr selectedTask = new SelectTask_vr();
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

    private bool checkIfCompatible(selectAgent_vr a)
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
            if (selectAgent_vr.currentlySelected.Count != 0)
            {
                foreach (selectAgent_vr s in selectAgent_vr.currentlySelected)
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

    public bool OnTrigged()
    {
        bool is_compatible = false;
        if (selectAgent_vr.currentlySelected.Count != 0)
        {
            foreach (selectAgent_vr s in selectAgent_vr.currentlySelected)
            {
                is_compatible = checkIfCompatible(s);
            }
            if (is_compatible)
            {
                selectedTask.rightClickAndSelected = false;
                selectedTask = this;
                foreach (selectAgent_vr s in selectAgent_vr.currentlySelected)
                {
                    GameWorldSubscriber.sendUserInput("ASSIGN|" + s.agentName + "|" + taskName);
                }
                print("Agent has been selected, and right click has been pressed.");
                return true;
            }
            else
            {
                rightClickAndSelected = false;
                print("Right click has been clicked, but INCORRECT AGENT SELECTED!");
                return false;
            }

        }
        else
        {
            rightClickAndSelected = false;
            return false;
            print("Right cick has been clicked, but NO AGENT SELECTED!");
        }
    }

    private void OnGUI()
    {
        if (rightClickAndSelected)
        {
            GUI.Box(new Rect(mousex + mouse_offset, Screen.height - mousey - mouse_offset, 200, 50), "");
            if (GUI.Button(new Rect(mousex + mouse_offset, Screen.height - mousey - mouse_offset, 200, 50), "Assigned Agent to Task"))
            {
                foreach (selectAgent_vr s in selectAgent_vr.currentlySelected)
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

