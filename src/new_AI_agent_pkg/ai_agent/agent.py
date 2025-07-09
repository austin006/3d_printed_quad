# Imports from langchain
from langchain_core.messages import SystemMessage
from langchain_ollama import ChatOllama

# Imports from langgraph
from langgraph.graph import START, StateGraph, MessagesState
from langgraph.prebuilt import tools_condition, ToolNode

# Imports from Python standard library for state definition
from typing import TypedDict, List, Dict, Any, Optional

# Import tools
from tools import tools

# Define LLM with bound tools
llm = ChatOllama(model="llama3.2:3b", base_url="http://host.docker.internal:11434", temperature=0)
llm_with_tools = llm.bind_tools(tools)

# Load the system prompt from the file
with open("system_prompt.txt", "r", encoding="utf-8") as f:
    system_prompt = f.read()

# System message
sys_msg = SystemMessage(content=system_prompt)

# Define the structured state for the drone
class DroneState(TypedDict):
    messages: List[Dict[str, Any]]
    vehicle_status: str
    battery_level: int
    position: tuple[float, float, float] # (lat, lon, altitude)
    is_armed: bool
    
# Nodes
def assistant(state: DroneState): 
    """This node will receive the user command and the current state of the drone and decide which tool to call next."""
    return {"messages": [llm_with_tools.invoke([sys_msg] + state["messages"])]}

def update_state(state: DroneState):   
    """After a tool is executed, this node will update the state of the drone."""

    # Simulate updating the drone state
    # In a real application, this would involve calling the drone's API or SDK to get the current state
    state["vehicle_status"] = "active"  # Example status
    state["battery_level"] = 85  # Example battery level
    state["position"] = (37.7749, -122.4194, 100)  # Example position (lat, lon, altitude)
    state["is_armed"] = True  # Example armed status
    
    # Append a message to the state indicating the drone state has been updated
    state["messages"].append({"role": "system", "content": 
    f""" 
    Drone state has been updated. Here is the current state of the drone:
    - Status: {state['vehicle_status']}
    - Battery: {state['battery_level']}%
    - Position: {state['position']}
    - Armed: {state['is_armed']}
    """
    })
        
    return {"messages": state["messages"]}

# Build graph nodes
builder = StateGraph(DroneState)
builder.add_node("update_state", update_state)
builder.add_node("llm_assistant", assistant)
builder.add_node("tool_executor", ToolNode(tools))

# Add edges
builder.add_edge(START, "update_state")
builder.add_edge("update_state", "llm_assistant")
builder.add_conditional_edges(
    "llm_assistant",
    tools_condition,
    {
        # If the LLM function call response is to call a tool, route to "tool_executor"
        "tools": "tool_executor",
        # Otherwise, if it's a regular message, end the graph
        "__end__": "__end__"
    }
)
builder.add_edge("tool_executor", "update_state")

# Compile graph
graph = builder.compile()
