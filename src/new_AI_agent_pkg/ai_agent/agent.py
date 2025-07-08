from langchain_core.messages import SystemMessage
from langchain_ollama import ChatOllama

from langgraph.graph import START, StateGraph, MessagesState
from langgraph.prebuilt import tools_condition, ToolNode

# Import tools from the tools module
from tools import tools

# Define LLM with bound tools
llm = ChatOllama(model="llama3.2:3b", base_url="http://host.docker.internal:11434", temperature=0)
llm_with_tools = llm.bind_tools(tools)

# System message
sys_msg = SystemMessage(content="You are a helpful assistant tasked with writing performing arithmetic on a set of inputs.")

# Node
def planner(state: MessagesState): 
    """This node will receive the user command and the current state of the drone and decide which tool to call next."""
    return {"messages": [llm_with_tools.invoke([sys_msg] + state["messages"])]}

def state(state: MessagesState):   
    """After a tool is executed, this node will update the state of the drone."""
    # Get the latest message from the state
    latest_message = state["messages"][-1]
    # Update the state with the latest message
    return {"messages": state["messages"] + [latest_message]}

# Build graph nodes
builder = StateGraph(MessagesState)
builder.add_node("llm_planner", planner)
builder.add_node("tool_executor", ToolNode(tools))
builder.add_node("update_state", state)

# Add edges
builder.add_edge(START, "llm_planner")
builder.add_conditional_edges(
    "llm_planner",
    # If the latest message (result) from planner is a tool call -> tools_condition routes to tools
    # If the latest message (result) from planner is a not a tool call -> tools_condition routes to END
    tools_condition,
)
builder.add_edge("tool_executor", "update_state")
builder.add_edge("update_state", "planner")

# Compile graph
graph = builder.compile()
