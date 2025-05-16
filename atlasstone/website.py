from xml.etree.ElementTree import tostring
from flask import Flask, request, jsonify
import requests 
from flask import render_template
import json
import threading
import math
import tkinter as tk
from tkinter import ttk
import PID_loop

app = Flask(__name__)

# Global variable to access from Flask routes
atlasController = None

#===============================================================
# WEBSITE 
#===============================================================
@app.route("/")
def hello_world():
    return render_template("index.html")

@app.route('/orientationdata', methods=['POST'])
def handle_orientationdata():
    data = request.get_json()
    if not data:
        return jsonify({'status': 'error', 'message': 'No data received'}), 400
    if not all(key in data for key in ('alpha', 'beta', 'gamma')):
        return jsonify({'status': 'error', 'message': 'Missing alpha, beta, or gamma'}), 400
    if atlasController:  # Ensure atlasController is initialized
        shared_state = atlasController.get_shared_state()
        with shared_state.lock:
            shared_state.target_yaw = data.get("alpha")
            shared_state.target_pitch = data.get("beta")
            shared_state.target_roll = data.get("gamma")
    return jsonify({"status": "success"}), 200

def sign(num):
    if(num < 0):
        return -1
    else:
        return 1

#===============================================================
# APPLICATION MANAGER
#===============================================================
# Create a global variable to access from Flask routes
atlasController = None

if __name__ == '__main__':
    # Check if we're the main Flask process, not the reloader
    import os
    # WERKZEUG_RUN_MAIN environment variable is set by Flask for the main process
    # when using the reloader (it won't be set in the reloader/monitor process)
    is_main_process = os.environ.get('WERKZEUG_RUN_MAIN') == 'true' or not os.environ.get('FLASK_ENV') == 'development'
    
    # Only initialize hardware in the main process, not the reloader
    if is_main_process:
        # Initialize Atlas Controller
        atlasController = PID_loop.AtlasstoneController()
        shared_state = atlasController.get_shared_state()
        
        # Start Atlas controller in a separate thread
        atlasController_thread = threading.Thread(target=atlasController.run, daemon=True)
        atlasController_thread.start()
    
    # Run Flask in the main thread (this is a blocking call)
    # Note: We're disabling debug mode to prevent the reloader issue
    # If you need debug features, use debug=True but with use_reloader=False
    app.run(debug=True, use_reloader=False, host='0.0.0.0', port=5000, ssl_context='adhoc')

#===============================================================
# APPLICATION THREAD BLUEPRINT
#===============================================================
# 1. add data mutex
# 2. call class in different thread
# 3. function in class should recall itself
# (see tkinter example)
