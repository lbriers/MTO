from flask import Flask, request, jsonify, render_template
import threading
import PID_loop

app = Flask(__name__)

# Initialize the controller globally
atlasController = PID_loop.AtlasstoneController()
shared_state = atlasController.get_shared_state()

# Start the controller thread
atlasController_thread = threading.Thread(target=atlasController.run, daemon=True)
atlasController_thread.start()

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
    return -1 if num < 0 else 1

