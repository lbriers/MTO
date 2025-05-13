from xml.etree.ElementTree import tostring

from flask import Flask, request, jsonify
import requests 
from flask import render_template
import json
import threading
import math
import tkinter as tk
from tkinter import ttk

app = Flask(__name__)
ATLASSTONE_URL = "http://localhost:8000"
#===============================================================
# DATA LOCKS
#===============================================================

# Shared data and lock
gyrodata = {"alpha": 0, "beta": 0, "gamma": 0}
data_lock = threading.Lock()

#===============================================================
# WEBSITE 
#===============================================================

@app.route("/")
def hello_world():
    return render_template("index.html")

@app.route('/orientationdata', methods=['POST'])
def handle_orientationdata():
    try:
        data = request.get_json()

        if not data:
            return jsonify({'status': 'error', 'message': 'No data received'}), 400

        if not all(key in data for key in ('alpha', 'beta', 'gamma')):
            return jsonify({'status': 'error', 'message': 'Missing alpha, beta, or gamma'}), 400

        alpha = data.get('alpha')
        beta = data.get('beta')
        gamma = data.get('gamma')

        # Validate that values are numbers
        if not all(isinstance(x, (int, float)) for x in (alpha, beta, gamma)):
            return jsonify({'status': 'error', 'message': 'Alpha, beta, and gamma must be numeric'}), 400

        # Prepare the command for the Atlasstone controller
        command = {
            'roll': alpha,
            'pitch': beta,
            'yaw': gamma
        }

        # Send the command to the Atlasstone controller
        try:
            response = requests.post(ATLASSTONE_URL, json=command)  #Changed path to root
            response.raise_for_status()  # Raise HTTPError for bad responses (4xx or 5xx)
            return jsonify({'status': 'success', 'message': 'Command sent to Atlasstone'}), 200
        except requests.exceptions.RequestException as e:
            print(f"Error sending command to Atlasstone: {e}")
            return jsonify({'status': 'error', 'message': f'Failed to send command to Atlasstone: {e}'}), 500

    except json.JSONDecodeError:
        return jsonify({'status': 'error', 'message': 'Invalid JSON data'}), 400
    except Exception as e:
        print(f"Error processing orientation data: {e}")  # Log the error
        return jsonify({'status': 'error', 'message': f'Server error: {e}'}), 500

#===============================================================
# DRAWING APPLICATION
#===============================================================

def sign(num):
    if(num < 0):
        return -1
    else:
        return 1

#===============================================================
# APPLICATION MANAGER
#===============================================================

if __name__ == '__main__':

    # Flask server in seperate thread
    #tkinter_thread = threading.Thread(target=run_tkinter)
    #tkinter_thread.daemon = True  # Allow main thread to exit even if this is running
    #tkinter_thread.start()



    app.run(debug=True, host='0.0.0.0', port=5000, ssl_context='adhoc')



#===============================================================
# APPLICATION THREAD BLUEPRINT
#===============================================================
# 1. add data mutex
# 2. call class in different thread
# 3. function in class should recall itself
# (see tkinter example)
