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

    public class GameWorldSubscriber : MonoBehaviour {
        private static RosSocket rosSocket;
        private Dictionary<string, float> displayMessages;
        public Component agentMenu;
        public GameObject contructionAgentModel;
        public GameObject transportAgentModel;
        public GameObject contructionTaskModel;
        public GameObject transportTaskModel;
        public GameObject dartModel;
        public Material lineMaterial;
        public int UpdateTime = 1;
        public string agent_topic = "/gameworld/agents";
        private Dictionary<string, GameObject> agents = new Dictionary<string, GameObject>();
        private Dictionary<string, GameObject> construction_tasks = new Dictionary<string, GameObject>();
        private Dictionary<string, GameObject> transport_tasks = new Dictionary<string, GameObject>();
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


        // Use this for initialization
        void Start () {
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
            OnRightClick rc = plane.GetComponent<OnRightClick>();
            if (rc.clicked_off)
            {
                foreach (GameObject task in construction_tasks.Values)
                {
                    SelectTask selections = task.GetComponent<SelectTask>();
                    selections.rightClickAndSelected = false;
                }

                foreach (GameObject task in transport_tasks.Values)
                {
                    SelectTask selections = task.GetComponent<SelectTask>();
                    selections.rightClickAndSelected = false;
                }

                if (selectAgent.currentlySelected.Count != 0)
                {
                    selectAgent.DeselectAll();
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
                        selectAgent menu = addingAgent.AddComponent<selectAgent>();
                        menu.agentType = agent_message.agent_type;
                        BoxCollider box = addingAgent.AddComponent<BoxCollider>();
                        box.size = box.size * 1.3f;
                        NameDisplay name_display = addingAgent.AddComponent<NameDisplay>();
                        if (agent_message.agent_type == "ConstructionAgent")
                            name_display.colour = red;
                        else
                            name_display.colour = blue;
                        AgentInfo agent_info = addingAgent.AddComponent<AgentInfo>();
                        name_display.name = agent_message.name;
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
                    selectAgent menu = currentAgent.GetComponent<selectAgent>();
                    menu.agentCurrentTask = agent_message.current_task;
                    menu.currentAction = agent_message.current_action;
                    menu.agentCapabilities = getStringCompatibilities(agent_message.capabilities);
                    menu.movementSpeed = agent_message.movement_speed.ToString();
                    menu.isAtTask = agent_message.is_at_task.ToString();
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
                            NameDisplay name_display = addingConstructionTask.AddComponent<NameDisplay>();
                            name_display.name = PERM_CONTRUCTION_NAME;
                            name_display.colour = offRed;
                            BoxCollider box = addingConstructionTask.AddComponent<BoxCollider>();
                            box.size = box.size * 1.3f;
                            SelectTask taskSelect = addingConstructionTask.AddComponent<SelectTask>();
                            taskSelect.taskName = PERM_CONTRUCTION_NAME;
                            taskSelect.taskType = "Construction";
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
                    if (construction_task_message.is_complete)
                    {
                        construction_tasks.Remove(construction_task_message.name);
                        Destroy(currentConstructionTask);
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
                        // Makes the transport task end a child of the adding transport task.
                        // addingTransportTaskEnd.transform.parent = addingTransportTask.transform;
                        transport_tasks.Add(transport_task_message.name, addingTransportTask);
                        LineRenderer line = addingTransportTask.AddComponent<LineRenderer>();
                        line.material = lineMaterial;
                        NameDisplay name_display = addingTransportTask.AddComponent<NameDisplay>();
                        name_display.colour = offBlue;
                        line.SetPositions(new Vector3[2] { new Vector3(addingTransportTask.transform.position.x, 0.01f, addingTransportTask.transform.position.z),
                            new Vector3(addingTransportTaskEnd.transform.position.x, 0.01f,addingTransportTaskEnd.transform.position.z) });
                        line.startWidth = 0.05f;
                        line.endWidth = 0.5f;
                        name_display.name = transport_task_message.name;
                        BoxCollider box = addingTransportTask.AddComponent<BoxCollider>();
                        box.size = box.size * 1.3f;
                        SelectTask taskSelect = addingTransportTask.AddComponent<SelectTask>();
                        taskSelect.taskName = transport_task_message.name;
                        taskSelect.taskType = "Transport";

                    }
                }
                else
                {
                    GameObject currentTransportTask;
                    transport_tasks.TryGetValue(transport_task_message.name, out currentTransportTask);
                    try
                    {
                        currentTransportTask.transform.position = getVectFromMessage(transport_task_message.pose, new Vector3(0, 0.47f, 0));
                        LineRenderer line = currentTransportTask.GetComponent<LineRenderer>();
                        line.SetPosition(0, new Vector3(currentTransportTask.transform.position.x, 0.01f, currentTransportTask.transform.position.z));
                        if (transport_task_message.is_complete)
                        {
                            transport_tasks.Remove(transport_task_message.name);
                            Destroy(currentTransportTask);
                        }
                    } catch(MissingReferenceException e)
                    {
                        print("Transport task was destroyed, the task is still in the dictionary for some reasons... Deleting it again.");
                        transport_tasks.Remove(transport_task_message.name);
                    }
                }
                transport_task_message = null;
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
