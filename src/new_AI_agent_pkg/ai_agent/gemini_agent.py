# main.py

# Imports from Python standard library
import json
from typing import Annotated, Dict, Any
from langchain_core.tools import tool
from langchain_core.messages import SystemMessage, HumanMessage, ToolMessage

# Imports from langchain
from langchain_ollama import ChatOllama

# Imports from langgraph
from langgraph.graph import StateGraph, START, END
from langgraph.graph.message import add_messages
from langgraph.prebuilt import ToolNode

# --- 1. Simulated Drone Environment ---
# In a real application, this would be replaced by an SDK or API client
# that communicates with the drone. For this example, we use a simple
# dictionary to represent the drone's state.

SIMULATED_DRONE_STATE = {
    "vehicle_status": "landed",
    "battery_level": 100,
    "position": (40.2463, -111.6479, 0),  # (lat, lon, relative_altitude)
    "is_armed": False,
    "home_position": (40.2463, -111.6479, 0),
}

print("--- Initial Drone State ---")
print(json.dumps(SIMULATED_DRONE_STATE, indent=2))
print("---------------------------\n")


# --- 2. Tool Definitions ---
# These tools interact with our simulated drone environment.
# Each tool returns a string confirming the action taken, which will be
# passed back to the LLM.

@tool
def get_drone_status() -> dict:
    """
    Retrieves the current status of the drone, including status, battery, position, and armed state.
    """
    print("--- TOOL EXECUTED: get_drone_status ---")
    return SIMULATED_DRONE_STATE

@tool
def arm_drone() -> str:
    """
    Arms the drone's motors in preparation for takeoff. The drone must be landed.
    """
    print("--- TOOL EXECUTED: arm_drone ---")
    if SIMULATED_DRONE_STATE["vehicle_status"] == "landed":
        SIMULATED_DRONE_STATE["is_armed"] = True
        SIMULATED_DRONE_STATE["vehicle_status"] = "armed"
        return "Drone has been armed successfully. Ready for takeoff."
    else:
        return f"Cannot arm drone. Current status is: {SIMULATED_DRONE_STATE['vehicle_status']}."

@tool
def disarm_drone() -> str:
    """
    Disarms the drone's motors. This can only be done when the drone is landed.
    """
    print("--- TOOL EXECUTED: disarm_drone ---")
    if SIMULATED_DRONE_STATE["vehicle_status"] == "landed":
        SIMULATED_DRONE_STATE["is_armed"] = False
        return "Drone has been disarmed."
    else:
        return "Cannot disarm drone while it is not landed."

@tool
def takeoff(altitude: float) -> str:
    """
    Commands the drone to take off to a specified altitude in meters. The drone must be armed first.
    """
    print(f"--- TOOL EXECUTED: takeoff(altitude={altitude}) ---")
    if SIMULATED_DRONE_STATE["is_armed"]:
        SIMULATED_DRONE_STATE["vehicle_status"] = "in_air"
        lat, lon, _ = SIMULATED_DRONE_STATE["position"]
        SIMULATED_DRONE_STATE["position"] = (lat, lon, altitude)
        return f"Takeoff successful. Ascending to {altitude} meters."
    else:
        return "Cannot take off. The drone is not armed."

@tool
def land() -> str:
    """
    Commands the drone to land at its current location.
    """
    print("--- TOOL EXECUTED: land ---")
    if SIMULATED_DRONE_STATE["vehicle_status"] == "in_air":
        SIMULATED_DRONE_STATE["vehicle_status"] = "landing"
        # Simulate landing process
        lat, lon, _ = SIMULATED_DRONE_STATE["position"]
        SIMULATED_DRONE_STATE["position"] = (lat, lon, 0)
        SIMULATED_DRONE_STATE["vehicle_status"] = "landed"
        SIMULATED_DRONE_STATE["is_armed"] = False
        return "Landing command executed. The drone has landed and disarmed."
    else:
        return "Drone is already on the ground."

@tool
def return_to_launch() -> str:
    """
    Commands the drone to return to its home position and land.
    """
    print("--- TOOL EXECUTED: return_to_launch ---")
    if SIMULATED_DRONE_STATE["vehicle_status"] == "in_air":
        SIMULATED_DRONE_STATE["position"] = SIMULATED_DRONE_STATE["home_position"]
        SIMULATED_DRONE_STATE["vehicle_status"] = "landed"
        SIMULATED_DRONE_STATE["is_armed"] = False
        return "Returning to launch point and landing. Mission complete."
    else:
        return "Drone is not in the air."

# List of all tools for the agent
tools = [get_drone_status, arm_drone, disarm_drone, takeoff, land, return_to_launch]


# --- 3. LLM and Prompt Setup ---

# The system prompt instructs the LLM on its role, capabilities, and how to behave.
# It's crucial for guiding the agent's reasoning process.
system_prompt = """
You are a highly intelligent drone control assistant. Your name is "Aria".
Your purpose is to translate human language commands into executable drone operations.

You have access to a set of tools to control the drone and check its status.
When you receive a command, first assess the drone's current state by using the `get_drone_status` tool, unless you are certain of its state from recent actions.
Then, execute the necessary tool(s) to fulfill the command.

- Always confirm the drone's status before executing critical commands like `takeoff`.
- For multi-step commands (e.g., "Arm and take off"), execute the tools sequentially. Call the first tool, wait for the result, then call the next.
- Provide clear, concise feedback to the user after each action.
- If a command cannot be executed, explain why based on the drone's current state.
- Your personality is professional, efficient, and helpful.
"""

# Define the LLM. Using a local Ollama model.
# Make sure your local Ollama server is running.
llm = ChatOllama(model="llama3", temperature=0)

# Bind the tools to the LLM, so it knows what functions it can call.
llm_with_tools = llm.bind_tools(tools)


# --- 4. Graph State Definition ---
# This defines the structure of the data that flows through the graph.
# We use `MessagesState`, which is a standard LangGraph class that
# automatically manages a list of messages.

class AgentState(Dict):
    messages: Annotated[list, add_messages]


# --- 5. Graph Nodes ---
# These are the functions that will be executed as nodes in our graph.

def call_model(state: AgentState):
    """The primary node that calls the LLM. It decides whether to call a tool or respond to the user."""
    print("--- CALLING MODEL ---")
    messages = state['messages']
    # Add the system prompt to the message list for the LLM call
    conversation_with_prompt = [SystemMessage(content=system_prompt)] + messages
    response = llm_with_tools.invoke(conversation_with_prompt)
    # The response will be a new message, which we add to the state.
    # If it's a ToolMessage, the graph will route to the tool_executor.
    # If it's an AIMessage, the graph will end.
    return {"messages": [response]}

# The ToolNode is a pre-built LangGraph node that executes tools.
tool_executor = ToolNode(tools)


# --- 6. Graph Construction ---
# Here, we define the workflow of our agent.

builder = StateGraph(AgentState)

# Add the nodes to the graph
builder.add_node("agent", call_model)
builder.add_node("tool_executor", tool_executor)

# The entry point of the graph is the 'agent' node.
builder.add_edge(START, "agent")

# This conditional edge decides the next step after the agent node runs.
def should_continue(state: AgentState):
    """
    Determines the next step after the LLM call.
    - If the LLM's response contains tool calls, route to the 'tool_executor' node.
    - Otherwise, the LLM has responded directly to the user, so end the graph execution.
    """
    if state['messages'][-1].tool_calls:
        return "tool_executor"
    else:
        return END

builder.add_conditional_edges(
    "agent",
    should_continue,
    {
        "tool_executor": "tool_executor",
        END: END
    }
)

# After a tool is executed, its output is added to the message history,
# and we loop back to the 'agent' node to let the LLM decide what to do next.
builder.add_edge("tool_executor", "agent")

# Compile the graph into a runnable object.
graph = builder.compile()


# --- 7. Running the Graph ---
def run_agent(user_input: str):
    """Helper function to stream and display the agent's execution."""
    print(f"\n--- USER COMMAND: '{user_input}' ---\n")
    
    # The initial state is the user's first message.
    initial_state = {"messages": [HumanMessage(content=user_input)]}
    
    # Stream the events from the graph as they happen.
    for event in graph.stream(initial_state, stream_mode="values"):
        # The event is the full state dictionary. We print the last message added.
        last_message = event["messages"][-1]
        
        if isinstance(last_message, ToolMessage):
            print(f"--- TOOL RESULT: {last_message.name} ---")
            print(f"Output: {last_message.content}")
            print("--------------------------------------\n")
        elif last_message.tool_calls:
            # This is the AI's decision to call a tool, but not the result yet.
            print(f"--- LLM DECISION: Call Tool(s) ---")
            print(last_message.tool_calls)
            print("------------------------------------\n")
        else:
            # This is the final response from the AI
            print("--- FINAL RESPONSE ---")
            print(last_message.content)
            print("----------------------\n")
            
    # Print the final state of the drone for verification
    print("\n--- Final Drone State ---")
    print(json.dumps(SIMULATED_DRONE_STATE, indent=2))
    print("-------------------------\n")


if __name__ == "__main__":
    # Example 1: A simple status check
    # run_agent("What is the drone's status?")
    
    # Example 2: A multi-step command
    run_agent("Please arm the drone, then take off to 20 meters.")

    # Example 3: A command that should fail initially
    # run_agent("Take off now.")

    # Example 4: A full mission
    # run_agent("Arm the drone, take off to 50 meters, and then return to launch.")
