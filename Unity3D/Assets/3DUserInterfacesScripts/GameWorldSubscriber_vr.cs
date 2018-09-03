using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEngine.SceneManagement;
using System.Globalization;



/*void onMouseDown()
{
    //TODO something
}*/

namespace RosSharp.RosBridgeClient
{ 
    [RequireComponent(typeof(RosConnector))]

    public class GameWorldSubscriber_vr : MonoBehaviour {
        private static RosSocket rosSocket;
        private Dictionary<string, float> displayMessages;
        public Component agentMenu;
        public GameObject contructionAgentModel;
        public GameObject transportAgentModel;
        public GameObject contructionTaskModel;
        public GameObject transportTaskModel;
        public GameObject dartModel;
        public Material lineMaterial;
        public Camera camera;
        public int UpdateTime = 1;
        public string agent_topic = "/gameworld/agents";
        private Dictionary<string, GameObject> agents = new Dictionary<string, GameObject>();
        private Dictionary<string, GameObject> construction_tasks = new Dictionary<string, GameObject>();
        private Dictionary<string, GameObject> transport_tasks = new Dictionary<string, GameObject>();
        private Dictionary<string, GameObject> textValues = new Dictionary<string, GameObject>();
        public string construction_task_topic = "/gameworld/construct_tasks";
        public string transport_task_topic = "/gameworld/transport_tasks";
        // Custom message declarations
        private GameworldAgent agent_message;
        private GameworldConstructionTask construction_task_message;
        private GameworldTransportTask transport_task_message;
        // Message for updating the user on what's happening.
        public string ros_update_message_topic = "/gameworld/update_messages";
        private StandardString update_message;
        public StandardString message = new StandardString();
        public string game_update_topic = "/Unity3D/update_message";
        private Color red;
        private Color blue;
        private Color offBlue;
        private Color offRed;
        private static string inputID;
        public string user_input_topic = "/Unity3D/user_input";
        private static string updateID;
        private bool start_recieved = false;
        private bool finished_recieved = false;
        private float finished_time = 0f;
        public GameObject plane;
        public int text_size = 20;
        public GameObject rightController;
        public GameObject leftController;
        private List<int> agent_ids;
        private List<int> task_ids;


        // Use this for initialization
        void Start () {
            agent_ids = new List<int>();
            task_ids = new List<int>();
            camera = Camera.main;
            displayMessages = new Dictionary<string, float>();
            red = new Color(1f, 0f, 0f, 0.5f);
            blue = new Color(0f, 0f, 1f, 0.5f);
            offRed = new Color(1f, 0f, 0.3f, 0.5f);
            offBlue = new Color(0.3f, 0f, 1f, 0.5f);
            print("Starting ROSBRIDGE and ROSSHARP!");
		    rosSocket = GetComponent<RosConnector>().RosSocket;
            rosSocket.Subscribe(agent_topic, "gameworld_simulation/Agent", agentSubscriber, UpdateTime);
            rosSocket.Subscribe(construction_task_topic, "gameworld_simulation/ConstructionTask", constuctionTaskSubscriber, UpdateTime);
            rosSocket.Subscribe(transport_task_topic, "gameworld_simulation/TransportTask", transportationTaskSubscriber, UpdateTime);
            rosSocket.Subscribe(ros_update_message_topic, "std_msgs/String", updateMessageSubscriber, UpdateTime);
            updateID = rosSocket.Advertise(game_update_topic, "std_msgs/String");
            inputID = rosSocket.Advertise(user_input_topic, "std_msgs/String");
            message.data = "start";
            rosSocket.Publish(updateID, message);
            lineMaterial = Resources.Load("Material/tile_ceilingPanel_fx", typeof(Material)) as Material;
        }

        public static void sendUpdateMessage(string strMessage)
        {
            StandardString message = new StandardString();
            message.data = strMessage;
            rosSocket.Publish(updateID, message);
        }

        public static void sendUserInput(string strMessage)
        {
            StandardString message = new StandardString();
            message.data = strMessage;
            rosSocket.Publish(inputID, message);
        }



        private void OnGUI()
        {
            int message_size_x = 400;
            int message_size_y = 50;

            GUILayout.BeginArea(new Rect((Screen.width * 0.95f) - message_size_x,
                  (Screen.height * 0.9f) - (message_size_y * displayMessages.Count + 1),
                  message_size_x,
                  message_size_y * (displayMessages.Count + 1)));
            GUILayout.FlexibleSpace();
            GUILayout.Box("Messages");
            if ((displayMessages != null) && (displayMessages.Count > 0))
            {
                List<string> keys = new List<string>(displayMessages.Keys);
                foreach (string k in keys)
                {
                    if(displayMessages[k] < 0f)
                    {
                        displayMessages.Remove(k);
                    } else
                    {
                        try
                        {
                            if (GUILayout.Button(k))
                            {
                                displayMessages[k] = 0f;
                            }
                            displayMessages[k] = displayMessages[k] - 0.0001f;
                        }
                        catch (ArgumentException e)
                        {
                            print("Caught exception in the GUI");
                        }
                        
                    }
                }
                
            }
            GUILayout.EndArea();

            if (finished_recieved)
            {
                int box_size_x = 250;
                int box_size_y = 100;
                //GUI.backgroundColor = Color.blue;
                GUIStyle style = new GUIStyle();
                style.fontSize = 30;
                GUI.Box(new Rect((Screen.width/2)-(box_size_x/2), (Screen.height/2)-(box_size_y/2), box_size_x, box_size_y), "You have finished the scenario!");
                GUILayout.BeginArea(new Rect((Screen.width / 2) - (box_size_x / 2), (Screen.height / 2) - (box_size_y / 2), box_size_x, box_size_y));
                GUILayout.FlexibleSpace();
                GUILayout.Box("Finish Time: " + finished_time + " seconds.");
                
                if (GUILayout.Button("Restart?"))
                {
                    SceneManager.LoadScene(0);
                }
                if (GUILayout.Button("Quit"))
                {
#if UNITY_EDITOR
                    UnityEditor.EditorApplication.isPlaying = false;
#else
                    Application.Quit();
#endif
                }
                GUILayout.EndArea();
            }
        }



        // Update is called once per frame
        void FixedUpdate () {
            SteamVR_LaserPointer rLaserPointer = rightController.GetComponent<SteamVR_LaserPointer>();
            SteamVR_LaserPointer lLaserPointer = leftController.GetComponent<SteamVR_LaserPointer>();
            if(rLaserPointer.hit_game_object != null)
            {
                GameObject hitObject = rLaserPointer.hit_game_object;
                if (agent_ids.Contains(hitObject.GetInstanceID()))
                {
                    print("Hit an agent!");
                    selectAgent_vr hitAgent = hitObject.GetComponent<selectAgent_vr>();
                    if (rLaserPointer.triggered)
                    {
                        hitAgent.OnMouseDown();
                    }
                }

                if (task_ids.Contains(hitObject.GetInstanceID()))
                {
                    print("Hit a task");
                    SelectTask_vr selectedTask = hitObject.GetComponent<SelectTask_vr>();
                    if (rLaserPointer.triggered)
                    {
                        if (selectedTask.OnTrigged())
                        {
                            rLaserPointer.hit_target = true;
                        }
                    }
                        
                        
                }
                /*if (hitObject.Equals(plane))
                {
                    selectAgent_vr.DeselectAll();
                }*/
            }
            if(lLaserPointer.hit_game_object != null)
            {
                print(lLaserPointer.hit_game_object);
            }


            OnRightClick rc = plane.GetComponent<OnRightClick>();
            if (rc.clicked_off)
            {
                foreach (GameObject task in construction_tasks.Values)
                {
                    SelectTask_vr selections = task.GetComponent<SelectTask_vr>();
                    selections.rightClickAndSelected = false;
                }

                foreach (GameObject task in transport_tasks.Values)
                {
                    SelectTask_vr selections = task.GetComponent<SelectTask_vr>();
                    selections.rightClickAndSelected = false;
                }

                if (selectAgent_vr.currentlySelected.Count != 0)
                {
                    selectAgent_vr.DeselectAll();
                }
                rc.clicked_off = false;
            }

            if (!start_recieved)
            {
                rosSocket.Publish(updateID, message);
            }
            
            // Update agents through the update function
            if (agent_message != null)
            {
                if (!agents.ContainsKey(agent_message.name))
                {
                    GameObject addingAgent = new GameObject();
                    try
                    {
                        if (agent_message.agent_type == "ConstructionAgent")
                        {
                             addingAgent = Instantiate(contructionAgentModel, getVectFromMessage(agent_message.pose), getQuadFromMsg(agent_message.pose));
                        }
                        else
                        {
                             addingAgent = Instantiate(transportAgentModel, getVectFromMessage(agent_message.pose), getQuadFromMsg(agent_message.pose));
                        }
                        selectAgent_vr menu = addingAgent.AddComponent<selectAgent_vr>();
                        agent_ids.Add(addingAgent.GetInstanceID());
                        menu.agentType = agent_message.agent_type;
                        BoxCollider box = addingAgent.AddComponent<BoxCollider>();
                        box.size = box.size * 1.3f;
                        GameObject textObject = new GameObject();
                        textObject.transform.position = addingAgent.transform.position;
                        textObject.transform.LookAt(camera.transform);
                        Vector3 rot = textObject.transform.eulerAngles;
                        rot = new Vector3(rot.x, rot.y + 180, rot.z);
                        textObject.transform.rotation = Quaternion.Euler(rot);
                        TextMesh text_name = textObject.AddComponent<TextMesh>();
                        text_name.text = agent_message.name;
                        text_name.fontSize = text_size;
                        //TextMesh agent_name = addingAgent.AddComponent<TextMesh>();
                        if (agent_message.agent_type == "ConstructionAgent")
                            text_name.color = red;
                        else
                            text_name.color = blue;
                        textValues.Add(agent_message.name, textObject);
                        AgentInfo agent_info = addingAgent.AddComponent<AgentInfo>();
                        //name_display.name = agent_message.name;
                        menu.agentName = agent_message.name;
                        menu.agentCurrentTask = agent_message.current_task;
                        menu.currentAction = agent_message.current_action;
                        menu.agentCapabilities = getStringCompatibilities(agent_message.capabilities);
                        menu.movementSpeed = agent_message.movement_speed.ToString();
                        menu.isAtTask = agent_message.is_at_task.ToString();
                        agents.Add(agent_message.name, addingAgent);
                    } catch (ArgumentException e)
                    {
                        print("Some reason, the agent has already been added... " + e);
                        Destroy(addingAgent);
                        agents.Remove(agent_message.name);

                    }
                    
                }
                else
                {
                    GameObject currentAgent;
                    
                    agents.TryGetValue(agent_message.name, out currentAgent);
                    currentAgent.transform.position = getVectFromMessage(agent_message.pose);
                    currentAgent.transform.localRotation = getQuadFromMsg(agent_message.pose);
                    selectAgent_vr menu = currentAgent.GetComponent<selectAgent_vr>();
                    menu.agentCurrentTask = agent_message.current_task;
                    menu.currentAction = agent_message.current_action;
                    menu.agentCapabilities = getStringCompatibilities(agent_message.capabilities);
                    menu.movementSpeed = agent_message.movement_speed.ToString();
                    menu.isAtTask = agent_message.is_at_task.ToString();
                    GameObject objText;
                    textValues.TryGetValue(agent_message.name, out objText);
                    objText.transform.position = currentAgent.transform.position;
                }
                agent_message = null;
            }

            if (construction_task_message != null)
            {
                string PERM_CONTRUCTION_NAME = String.Copy(construction_task_message.name);
                if ((!construction_tasks.ContainsKey(construction_task_message.name)))
                {
                    try
                    {
                        if (!construction_task_message.is_complete)
                        {
                            GameObject addingConstructionTask = Instantiate(contructionTaskModel, getVectFromMessage(construction_task_message.pose, new Vector3(0, 1f, 0)), getQuadFromMsg(construction_task_message.pose));
                            construction_tasks.Add(PERM_CONTRUCTION_NAME, addingConstructionTask);
                            //NameDisplay name_display = addingConstructionTask.AddComponent<NameDisplay>();
                            //name_display.name = PERM_CONTRUCTION_NAME;
                            task_ids.Add(addingConstructionTask.GetInstanceID());
                            BoxCollider box = addingConstructionTask.AddComponent<BoxCollider>();
                            box.size = box.size * 1.3f;
                            SelectTask_vr taskSelect = addingConstructionTask.AddComponent<SelectTask_vr>();
                            taskSelect.taskName = PERM_CONTRUCTION_NAME;
                            taskSelect.taskType = "Construction";

                            GameObject textObject = new GameObject();
                            textObject.transform.position = addingConstructionTask.transform.position;
                            textObject.transform.LookAt(camera.transform);
                            Vector3 rot = textObject.transform.eulerAngles;
                            rot = new Vector3(rot.x, rot.y + 180, rot.z);
                            textObject.transform.rotation = Quaternion.Euler(rot);
                            TextMesh text_name = textObject.AddComponent<TextMesh>();
                            text_name.text = construction_task_message.name;
                            text_name.fontSize = text_size;
                            text_name.color = offRed;
                            textValues.Add(construction_task_message.name, textObject);
                        }
                    } 
                    catch (ArgumentException e)
                    {
                        print("Some reason, the contruction task has already been added... " + e);
                    }

                }
                else
                {
                    GameObject currentConstructionTask;
                    construction_tasks.TryGetValue(construction_task_message.name, out currentConstructionTask);
                    GameObject objText;
                    textValues.TryGetValue(construction_task_message.name, out objText);
                    objText.transform.position = currentConstructionTask.transform.position;
                    if (construction_task_message.is_complete)
                    {
                        construction_tasks.Remove(construction_task_message.name);
                        Destroy(currentConstructionTask);
                        textValues.Remove(construction_task_message.name);
                        Destroy(objText);
                    }
                }
                construction_task_message = null;


            }

            if (transport_task_message != null)
            {
                if (!transport_tasks.ContainsKey(transport_task_message.name))
                {
                    if (!transport_task_message.is_complete)
                    {
                        GameObject addingTransportTask = Instantiate(transportTaskModel, getVectFromMessage(transport_task_message.pose, new Vector3(0, 0.47f, 0)), getQuadFromMsg(transport_task_message.pose));
                        GameObject addingTransportTaskEnd = Instantiate(dartModel, getVectFromMessage(transport_task_message.end_pose, new Vector3(0, 1.252f, 0)), Quaternion.Euler(90, 0, 0));
                        addingTransportTaskEnd.transform.localScale = new Vector3(200, 200, 200);
                        task_ids.Add(addingTransportTask.GetInstanceID());
                        // Makes the transport task end a child of the adding transport task.
                        // addingTransportTaskEnd.transform.parent = addingTransportTask.transform;
                        transport_tasks.Add(transport_task_message.name, addingTransportTask);
                        LineRenderer line = addingTransportTask.AddComponent<LineRenderer>();
                        line.material = lineMaterial;
                        //NameDisplay name_display = addingTransportTask.AddComponent<NameDisplay>();
                        //name_display.colour = offBlue;
                        line.SetPositions(new Vector3[2] { new Vector3(addingTransportTask.transform.position.x, 0.01f, addingTransportTask.transform.position.z),
                            new Vector3(addingTransportTaskEnd.transform.position.x, 0.01f,addingTransportTaskEnd.transform.position.z) });
                        line.startWidth = 0.05f;
                        line.endWidth = 0.5f;
                        //name_display.name = transport_task_message.name;
                        BoxCollider box = addingTransportTask.AddComponent<BoxCollider>();
                        box.size = box.size * 1.3f;
                        SelectTask_vr taskSelect = addingTransportTask.AddComponent<SelectTask_vr>();
                        taskSelect.taskName = transport_task_message.name;
                        taskSelect.taskType = "Transport";

                        GameObject textObject = new GameObject();
                        textObject.transform.position = addingTransportTask.transform.position;
                        textObject.transform.LookAt(camera.transform);
                        Vector3 rot = textObject.transform.eulerAngles;
                        rot = new Vector3(rot.x, rot.y + 180, rot.z);
                        textObject.transform.rotation = Quaternion.Euler(rot);
                        TextMesh text_name = textObject.AddComponent<TextMesh>();
                        text_name.text = transport_task_message.name;
                        text_name.fontSize = text_size ;
                        text_name.color = offBlue;
                        textValues.Add(transport_task_message.name, textObject);
                    }
                }
                else
                {
                    GameObject currentTransportTask;
                    transport_tasks.TryGetValue(transport_task_message.name, out currentTransportTask);

                    GameObject objText;
                    textValues.TryGetValue(transport_task_message.name, out objText);
                    objText.transform.position = currentTransportTask.transform.position;

                    try
                    {
                        currentTransportTask.transform.position = getVectFromMessage(transport_task_message.pose, new Vector3(0, 0.47f, 0));
                        LineRenderer line = currentTransportTask.GetComponent<LineRenderer>();
                        line.SetPosition(0, new Vector3(currentTransportTask.transform.position.x, 0.01f, currentTransportTask.transform.position.z));
                        if (transport_task_message.is_complete)
                        {
                            transport_tasks.Remove(transport_task_message.name);
                            Destroy(currentTransportTask);
                            textValues.Remove(transport_task_message.name);
                            Destroy(objText);
                        }
    
                    } catch(MissingReferenceException e)
                    {
                        print("Transport task was destroyed, the task is still in the dictionary for some reasons... Deleting it again.");
                        transport_tasks.Remove(transport_task_message.name);
                    }
                }
                transport_task_message = null;
            }

            if (textValues.Count > 0)
            {
                List<string> keys = new List<string>(textValues.Keys); 
                foreach (string k in keys)
                {
                    textValues[k].transform.LookAt(Camera.main.transform);
                    Vector3 rot = textValues[k].transform.eulerAngles;
                    rot = new Vector3(-rot.x, rot.y + 180, rot.z);
                    textValues[k].transform.rotation = Quaternion.Euler(rot);
                }
            }
            
            if (construction_tasks.Count != 0)
            {
                foreach (GameObject task in construction_tasks.Values)
                {
                    task.transform.localEulerAngles = new Vector3(45, task.transform.localEulerAngles.y + 1f, 90);
                }
            }
        }

        private string getStringCompatibilities(int[] compatibilities)
        {
            string strCompatibilities = "";
            foreach (int x in compatibilities)
            {
                string ability;
                switch (x)
                {
                    case 1:
                        ability = "Flight";
                        break;
                    case 2:
                        ability = "Drive";
                        break;
                    case 3:
                        ability = "Pickup";
                        break;
                    case 4:
                        ability = "Construct";
                        break;
                    case 5:
                        ability = "Clean";
                        break;
                    case 6:
                        ability = "Transport";
                        break;
                    default:
                        ability = "None";
                        break;
                }
                strCompatibilities += (ability + ", ");
            }

            return strCompatibilities;
        }

        // TODO Actually fill this with usefull stuff!
        private void agentSubscriber(Message message)
        {
            GameworldAgent mes = (GameworldAgent)message;
            print("Got an agent message!");
            if (mes.Equals(agent_message))
            {
                print("Agent message recieved was the same as the last one");
            }
            agent_message = mes;


        }

        private Quaternion getQuadFromMsg(GeometryPose pose)
        {
            return new Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        }
        private Vector3 getVectFromMessage(GeometryPose pose)
        {
            Vector3 scale = new Vector3(0.1f, 0.1f, 0.1f);
            Vector3 rosPos = new Vector3(pose.position.x, pose.position.z, pose.position.y);
            //rosPos.Scale(scale);
            return rosPos;
        }

        private Vector3 getVectFromMessage(GeometryPose pose, Vector3 vec)
        {
            Vector3 scale = new Vector3(0.1f, 0.1f, 0.1f);
            Vector3 rosPos = new Vector3(pose.position.x, pose.position.z, pose.position.y);
            //rosPos.Scale(scale);
            return rosPos + vec;
        }

        private void transportationTaskSubscriber(Message message)
        {
            transport_task_message = (GameworldTransportTask)message;
            print("Got a transportation task message! \t " 
                + transport_task_message.end_pose.position.x 
                + ", " + transport_task_message.end_pose.position.y
                + ", " + transport_task_message.end_pose.position.z);
        }

        private void constuctionTaskSubscriber(Message message)
        {
            construction_task_message = (GameworldConstructionTask)message;
            print("Got a construction task message!");
        }

        private void transportTaskSubscriber(Message message)
        {
            transport_task_message = (GameworldTransportTask)message;
            print("Got a transport task message!");
        }

        private void updateMessageSubscriber(Message message)
        {
            update_message = (StandardString)message;
            print("Recieved Update Message: " + update_message.data);
            string[] splitString = update_message.data.Split('|');
            if (update_message.data == "start command recieved")
            {
                start_recieved = true;
            }
            else if (update_message.data == "finished")
            {
                StandardString finish_message = new StandardString();
                finished_recieved = true;
                finish_message.data = "finished confirmed";
                rosSocket.Publish(updateID, finish_message);
            }
            else if (splitString[0] == "end time")
            {
                finished_time = float.Parse(splitString[1], CultureInfo.InvariantCulture.NumberFormat);
            }
            else if (splitString[0] == "display message")
            {
                if (!displayMessages.ContainsKey(splitString[1]))
                {
                    displayMessages.Add(splitString[1], 5f);
                }
            }
        }
    }
}
