#!/usr/bin/env python3
import sys
import os
import subprocess

def main():
    # Find the source directory where venv actually lives
    # The package is installed, but venv is in source
    src_path = os.path.expanduser("~/ros2_workspaces/3d_printed_quad/src/ai_agent")
    venv_path = os.path.join(src_path, "venv", "lib", "python3.12", "site-packages")
    
    if os.path.exists(venv_path):
        sys.path.insert(0, venv_path)
        print(f"✓ Added venv to path: {venv_path}")
    else:
        print(f"✗ Error: Virtual environment not found at {venv_path}")
        print("Please ensure you've created the venv in the source directory:")
        print(f"  cd {src_path}")
        print("  python3 -m venv venv")
        print("  source venv/bin/activate")
        print("  pip install -r requirements.txt")
        sys.exit(1)
    
    # Now import and run the agent
    try:
        from ai_agent.agent import main as agent_main
        agent_main()
    except ImportError as e:
        print(f"✗ Import error: {e}")
        print("Make sure all dependencies are installed in the virtual environment")
        sys.exit(1)

if __name__ == '__main__':
    main()