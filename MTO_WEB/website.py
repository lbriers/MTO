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
ATLASSTONE_URL = "http://localhost:8000"
#===============================================================
# DATA LOCKS
#===============================================================

# Shared data and lock
#gyrodata = {"alpha": 0, "beta": 0, "gamma": 0}
#data_lock = threading.Lock()

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

        return jsonify({"status": "success"}), 200 # Add return statement

def sign(num):
    if(num < 0):
        return -1
    else:
        return 1

#===============================================================
# APPLICATION MANAGER
#===============================================================

def start_server():
    app.run(debug=True, host='0.0.0.0', port=5000, ssl_context='adhoc')


if __name__ == '__main__':

    # Flask server in seperate thread
    #tkinter_thread = threading.Thread(target=run_tkinter)
    #tkinter_thread.daemon = True  # Allow main thread to exit even if this is running
    #tkinter_thread.start()

    atlasController = PID_loop.AtlasstoneController()
    shared_state = atlasController.get_shared_state()
    #atlasController.setWebsiteData(gyrodata, data_lock)
    atlasController_thread = threading.Thread(target=atlasController.run(), daemon=True)

    atlasController_thread.start()
    



    app.run(debug=True, host='0.0.0.0', port=5000, ssl_context='adhoc')



#===============================================================
# APPLICATION THREAD BLUEPRINT
#===============================================================
# 1. add data mutex
# 2. call class in different thread
# 3. function in class should recall itself
# (see tkinter example)
