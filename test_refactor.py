import subprocess
import os
import sys

os.environ["SDL_VIDEODRIVER"] = "dummy"

try:
    print("Starting simulation...")
    # Run for 5 seconds to verify initialization and basic loop
    subprocess.run([sys.executable, "main.py"], timeout=5, check=True)
except subprocess.TimeoutExpired:
    print("Simulation ran for 5 seconds successfully (timeout expired).")
except subprocess.CalledProcessError as e:
    print(f"Simulation failed with exit code {e.returncode}")
    sys.exit(1)
except Exception as e:
    print(f"An error occurred: {e}")
    sys.exit(1)
