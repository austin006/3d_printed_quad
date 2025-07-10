from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # First ensure Ollama is running
    ollama_process = ExecuteProcess(
        cmd=['ollama', 'serve'],
        output='screen',
        name='ollama_server'
    )
    
    # Run the agent with the virtual environment
    agent_process = ExecuteProcess(
        cmd=['bash', '-c', 
             'cd ~/your_ws/src/ai_agent && '
             'source venv/bin/activate && '
             'source ~/your_ws/install/setup.bash && '
             'export PYTHONPATH=$PYTHONPATH:~/your_ws/src/ai_agent && '
             'python3 ai_agent/agent.py'],
        output='screen',
        name='ai_agent'
    )
    
    return LaunchDescription([
        ollama_process,
        agent_process
    ])