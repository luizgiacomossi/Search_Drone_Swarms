import tkinter as tk
from tkinter import ttk
import queue

def run_dashboard(data_queue):
    """
    Runs the Tkinter dashboard in a separate process.
    
    Args:
        data_queue (multiprocessing.Queue): Queue to receive data from the main simulation.
    """
    root = tk.Tk()
    root.title("Drone Swarm Dashboard")
    root.geometry("600x500")
    
    # --- Styles ---
    style = ttk.Style()
    style.configure("Bold.TLabel", font=("Helvetica", 12, "bold"))
    
    # --- Experiment Info Section ---
    info_frame = ttk.LabelFrame(root, text="Experiment Information", padding=(10, 10))
    info_frame.pack(fill="x", padx=10, pady=5)
    
    # Variables
    var_algo = tk.StringVar(value="Waiting...")
    var_swarm = tk.StringVar(value="-")
    var_obstacles = tk.StringVar(value="-")
    var_run = tk.StringVar(value="-")
    
    # Grid layout for info
    ttk.Label(info_frame, text="Algorithm:", style="Bold.TLabel").grid(row=0, column=0, sticky="w")
    ttk.Label(info_frame, textvariable=var_algo).grid(row=0, column=1, sticky="w", padx=10)
    
    ttk.Label(info_frame, text="Drone Count:", style="Bold.TLabel").grid(row=1, column=0, sticky="w")
    ttk.Label(info_frame, textvariable=var_swarm).grid(row=1, column=1, sticky="w", padx=10)
    
    ttk.Label(info_frame, text="Obstacles:", style="Bold.TLabel").grid(row=2, column=0, sticky="w")
    ttk.Label(info_frame, textvariable=var_obstacles).grid(row=2, column=1, sticky="w", padx=10)
    
    ttk.Label(info_frame, text="Current Run:", style="Bold.TLabel").grid(row=3, column=0, sticky="w")
    ttk.Label(info_frame, textvariable=var_run).grid(row=3, column=1, sticky="w", padx=10)
    
    # --- Status Section ---
    status_frame = ttk.LabelFrame(root, text="Real-Time Status", padding=(10, 10))
    status_frame.pack(fill="x", padx=10, pady=5)
    
    var_time = tk.StringVar(value="0.00 s")
    var_progress = tk.StringVar(value="0%")
    
    ttk.Label(status_frame, text="Simulation Time:").grid(row=0, column=0, sticky="w")
    ttk.Label(status_frame, textvariable=var_time, font=("Courier", 14)).grid(row=0, column=1, sticky="w", padx=10)
    
    ttk.Label(status_frame, text="Completion:").grid(row=1, column=0, sticky="w")
    ttk.Label(status_frame, textvariable=var_progress).grid(row=1, column=1, sticky="w", padx=10)

    var_coverage = tk.StringVar(value="0.0%")
    ttk.Label(status_frame, text="Map Coverage:").grid(row=2, column=0, sticky="w")
    ttk.Label(status_frame, textvariable=var_coverage).grid(row=2, column=1, sticky="w", padx=10)
    
    # --- History Section ---
    history_frame = ttk.LabelFrame(root, text="Run History", padding=(10, 10))
    history_frame.pack(fill="both", expand=True, padx=10, pady=5)
    
    history_list = tk.Listbox(history_frame, height=10)
    history_list.pack(fill="both", expand=True, side="left")
    
    scrollbar = ttk.Scrollbar(history_frame, orient="vertical", command=history_list.yview)
    scrollbar.pack(side="right", fill="y")
    history_list.config(yscrollcommand=scrollbar.set)
    
    # --- Update Loop ---
    def poll_queue():
        try:
            # Process all available messages to stay up to date
            while True:
                data = data_queue.get_nowait()
                
                # Update variables if keys exist
                if 'algorithm' in data: var_algo.set(data['algorithm'])
                if 'swarm_size' in data: var_swarm.set(str(data['swarm_size']))
                if 'obstacles' in data: var_obstacles.set(str(data['obstacles']))
                if 'run_index' in data: var_run.set(str(data['run_index']))
                
                if 'time' in data: var_time.set(f"{data['time']:.2f} s")
                if 'progress' in data: var_progress.set(f"{data['progress']*100:.1f}%")
                if 'coverage' in data: var_coverage.set(f"{data['coverage']*100:.1f}%")
                
                if 'history_entry' in data:
                    history_list.insert(0, data['history_entry']) # Insert at top
                    
        except queue.Empty:
            pass
        finally:
            # Check again after 100ms
            root.after(100, poll_queue)
            
    # Start polling
    root.after(100, poll_queue)
    
    # Start loop
    try:
        root.mainloop()
    except KeyboardInterrupt:
        root.destroy()
