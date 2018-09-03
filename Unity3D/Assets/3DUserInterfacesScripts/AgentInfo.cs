using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// This class is just used to store the identifiable data for each agent.
public class AgentInfo : MonoBehaviour {
    public string name;
    public string agent_type;

    public AgentInfo(string agent_name, string agentType)
    {
        this.name = agent_name;
        this.agent_type = agentType;
    }

}
