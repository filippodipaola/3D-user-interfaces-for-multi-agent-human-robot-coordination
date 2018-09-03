using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class selectAgent : MonoBehaviour {

    public string agentName;
    public string agentCurrentTask;
    public string agentCapabilities;
    public string currentAction;
    public string movementSpeed;
    public string isAtTask;
    public string agentType;
    private bool mouseHover = false;
    private bool timeOn = false;
    private float timer = 0.5f;
    public static HashSet<selectAgent> allMySelectables = new HashSet<selectAgent>();
    public static HashSet<selectAgent> currentlySelected = new HashSet<selectAgent>();
    public Material unselectedMaterial;
    public Material selectedMaterial;
    private Renderer myRenderer;
    private float mouse_offset;
    void OnMouseOver()
    {
        mouseHover = true;
    }

    private void OnMouseExit()
    {
        mouseHover = false;
    }

    private void OnMouseDown()
    {
        print("I've been clicked!");
        DeselectAll();
        OnSelect();

    }

    public void OnSelect()
    {
        currentlySelected.Add(this);
        myRenderer.material = selectedMaterial;
        // NameDisplay name = gameObject.GetComponent<NameDisplay>();
    }

    public void OnDeselect()
    {
        myRenderer.material = unselectedMaterial;
    }


    public static void DeselectAll()
    {
        foreach (selectAgent selectable in currentlySelected)
        {
            selectable.OnDeselect();
        }
        currentlySelected.Clear();
    }

    private void OnGUI()
    {
        if (mouseHover || timeOn)
        {
            //print("X: "+ Input.mousePosition.x + ", Y: "+ Input.mousePosition.y);
            GUI.Box(new Rect(Input.mousePosition.x + mouse_offset, Screen.height - Input.mousePosition.y - mouse_offset, 250, 245), "Agent Menu");
            GUI.TextField(new Rect(Input.mousePosition.x + mouse_offset + 10, Screen.height - Input.mousePosition.y - mouse_offset + 30, 220, 25), "Agent Name: " + agentName);  // Agent name
            GUI.TextField(new Rect(Input.mousePosition.x + mouse_offset + 10, Screen.height - Input.mousePosition.y - mouse_offset + 40 + (25 * 1), 220, 25), "Current Task: " + agentCurrentTask);  // Agent task
            GUI.TextField(new Rect(Input.mousePosition.x + mouse_offset + 10, Screen.height - Input.mousePosition.y - mouse_offset + 50 + (25 * 2), 220, 25), "Capabilities: " + agentCapabilities);  // Agent capabilities
            GUI.TextField(new Rect(Input.mousePosition.x + mouse_offset + 10, Screen.height - Input.mousePosition.y - mouse_offset + 60 + (25 * 3), 220, 25), "Current Action: " + currentAction);  // Agent action
            GUI.TextField(new Rect(Input.mousePosition.x + mouse_offset + 10, Screen.height - Input.mousePosition.y - mouse_offset + 70 + (25 * 4), 220, 25), "Movement Speed: " + movementSpeed);  // Agent speed
            GUI.TextField(new Rect(Input.mousePosition.x + mouse_offset + 10, Screen.height - Input.mousePosition.y - mouse_offset + 80 + (25 * 5), 220, 25), "At the task: " + isAtTask);  // Agent is at task
        }
        
    }
    // Use this for initialization
    void Start () {
        myRenderer = GetComponent<Renderer>();
        
        //print("All " + Resources.FindObjectsOfTypeAll(typeof(Material)).Length);
        unselectedMaterial = Resources.Load("Material/prop_floorBot_mat", typeof(Material)) as Material;
        selectedMaterial = Resources.Load("Material/tile_ceilingPanel_fx", typeof(Material)) as Material;
        print(unselectedMaterial);
        mouse_offset = 10f;
    }
	
	// Update is called once per frame
	void Update () {
		if(!mouseHover)
        {
            if (timer < 0f)
            {
                timeOn = false;
            } else
            {
                timer -= Time.deltaTime;
                timeOn = true;
            }
        }

        if(mouseHover)
        {
            timer =0.5f;
        }

	}
}
