from flask import Flask, request, jsonify
from flask import render_template

app = Flask(__name__)

@app.route("/")
def hello_world():
    return render_template("index.html")

@app.route('/orientationdata', methods=['POST'])
def handle_orientationdata():
    data = request.get_json()

    if data:
        alpha = data.get('alpha')
        beta = data.get('beta')
        gamma = data.get('gamma')

        #print(f"Received orientation data: Alpha={alpha}, Beta={beta}, Gamma={gamma}")

        # Process the data (e.g., store in a database)
        # ...

        return jsonify({'status': 'success', 'message': 'Orientation data received'}), 200
    else:
        return jsonify({'status': 'error', 'message': 'No data received'}), 400

if __name__ == '__main__':
 app.run(debug=True, host='0.0.0.0', port=5000, ssl_context='adhoc')
