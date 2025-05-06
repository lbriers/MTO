
 // Get references to the alpha, beta, and gamma display elements.
const alphaElement = document.getElementById("alpha");
const betaElement = document.getElementById("beta");
const gammaElement = document.getElementById("gamma");

// get buttons
const button_trim_left = document.getElementById("button_trim_left");
const button_trim_right = document.getElementById("button_trim_right");

// Get the sensor support label element.
const sensorSupportLabel = document.getElementById("sensorSupportLabel");
const enableGyroButton = document.getElementById("infoToggle");



/***************************************
	* BACKEND COMMUNICATION CODE
***************************************/
const sendDataToServer = (data) => {
	fetch("/orientationdata", {
		method: "POST",
		headers: {
			"Content-Type": "application/json",
		},
		body: JSON.stringify(data),
	})
		.then((response) => {
			if (!response.ok) {
				throw new Error("Network response was not ok");
			}
			console.log("Data sent successfully");
		})
		.catch((error) => {
			console.error("There was a problem sending the data:", error);
		});
};


/***************************************
	* SENSOR SUPPORTED LABEL
***************************************/
// Function to update the sensor support label.
const updateSensorSupportLabel = (supported) => {
	if (supported) {
		sensorSupportLabel.textContent = "Enable tilt controls";
		sensorSupportLabel.classList.add("supported");
	} else {
		sensorSupportLabel.textContent =
			"Tilt controls NOT Supported!";
		sensorSupportLabel.classList.add("not-supported");
	}
};


/***************************************
	* SENSOR CODE
***************************************/
// Function to convert quaternion to Euler angles (in degrees).
const getEulerAngles = (q) => {
	const toDegrees = (rad) => (rad * 180) / Math.PI;

	// Convert quaternion to Euler angles (yaw, pitch, roll)
	const sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3]);
	const cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
	const alpha = toDegrees(Math.atan2(sinr_cosp, cosr_cosp)); // Yaw (alpha)

	const sinp = 2 * (q[0] * q[2] - q[3] * q[1]);
	let beta = toDegrees(Math.asin(sinp)); 					  // Pitch (beta)
	if (Math.abs(sinp) >= 1) {
		beta = toDegrees(Math.sign(sinp) * Math.PI / 2); 	  // Use 90 degrees if out of range
	}

	const siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2]);
	const cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
	const gamma = toDegrees(Math.atan2(siny_cosp, cosy_cosp));  // Roll (gamma)


	return {
		alpha: alpha,
		beta: beta,
		gamma: gamma,
	};
};

// Function to start the sensor and handle data processing.
const startSensor = (sensor) => {
	let lastSent = 0;
	const interval = 1000 / 30; // Interval in milliseconds for ~30Hz

	sensor.addEventListener("reading", () => {
		const now = Date.now();
		if (now - lastSent >= interval) {
			// The quaternion values represent the device's orientation
			const quaternion = sensor.quaternion;

			if(enableGyroButton.checked == true){
				// Convert quaternion to Euler angles (degrees)
				const euler = getEulerAngles(quaternion);
				const alpha = euler.alpha; // Yaw (rotation around Z axis)
				const beta = euler.beta;  // Pitch (rotation around X axis)
				const gamma = euler.gamma; // Roll (rotation around Y axis)
			}

			alphaElement.textContent = alpha.toFixed(2); 				  // Display with 2 decimal places
			betaElement.textContent = beta.toFixed(2); 				  // Display with 2 decimal places
			gammaElement.textContent = gamma.toFixed(2); 				  // Display with 2 decimal places

			const orientationData = {
				alpha: alpha,
				beta: beta,
				gamma: gamma,
			};

			sendDataToServer(orientationData);
			lastSent = now;
		}
	});

	sensor.addEventListener("error", (event) => {
		console.error("Sensor error:", event.error.message);
	});

	sensor.start();
};

// INITIALIZE SENSOR
if ("AbsoluteOrientationSensor" in window) {
	let sensor = new AbsoluteOrientationSensor({
		frequency: 60,  // Adjust frequency as needed (Hz)
		referenceFrame: "device", // Or 'screen'
	});

	// Request permission (if needed - check browser support)
	if (typeof sensor.requestPermission === "function") {
		sensor
			.requestPermission()
			.then((permissionState) => {
				if (permissionState === "granted") {
					updateSensorSupportLabel(true);
					startSensor(sensor);
				} else {
					updateSensorSupportLabel(false);
					console.warn("Permission to access sensor was denied");
				}
			})
			.catch((error) => {
				updateSensorSupportLabel(false);
				console.error(error);
			});
	} else {
		updateSensorSupportLabel(true);
		startSensor(sensor);
	}
} else {
	updateSensorSupportLabel(false);
	console.warn("AbsoluteOrientationSensor not supported");
}


/***************************************
	* JOYSTICK CODE
***************************************/
// Initialize variables for mouse interaction.
let xco = 0;
let yco = 0;
let mousedown = false;

maxDistance = 80;

// Get the canvas element and its 2D rendering context.
let canvas = document.getElementById("joystickCanvas");
const ctx = canvas.getContext("2d");

// Set canvas dimensions in px
const size = 500
canvas.width = size;
canvas.height = size; 

//initialize output variables for the joystick output
let outX, outY;
let trim_gamma = 0;

// DRAW JOYSTICK AND CALCULATE ALPHA, BETA
const drawJoystick = (x, y) => {
	//Clear the canvas
    	ctx.clearRect(0, 0, canvas.width, canvas.height);

    	// Draw outer circle
    	ctx.beginPath();
    	ctx.arc(canvas.width / 2, canvas.height / 2, size*0.4, 0, 2 * Math.PI);
    	ctx.stroke();

    	let dx = x - canvas.width / 2;
    	let dy = y - canvas.height / 2;
    	let angle = Math.atan2(dy, dx);
    	let distance = Math.sqrt(dx ** 2 + dy ** 2);

    	if (distance > size*0.4) {
        	distance = size*0.4;
    	}

    	outX = canvas.width / 2 + distance * Math.cos(angle);
    	outY = canvas.height / 2 + distance * Math.sin(angle);
	
    	// Draw joystick handle
    	ctx.beginPath();
    	ctx.arc(
		outX, outY, 
        	size*0.2,
        	0,
        	2 * Math.PI
	);
    	ctx.fill();

	//Convert joystick position
	if(enableGyroButton.checked == false){
		alpha = (distance * Math.cos(angle)/size)*112.5;
		beta = -(distance*Math.sin(angle)/size)*112.5;

		//display values
		alphaElement.textContent = alpha.toFixed(2); 		// Display with 2 decimal places
		betaElement.textContent = beta.toFixed(2); 		// Display with 2 decimal places
		gammaElement.textContent = trim_gamma; 		  	// Display with 2 decimal places
		const orientationData = {
			alpha: alpha,
			beta: beta,
			gamma: trim_gamma,
		};
		sendDataToServer(orientationData);
	}
};

// Initialize joystick position
drawJoystick(canvas.width / 2, canvas.height / 2);// Function to send orientation data to the server.


/***************************************
	* INPUT LISTENERS 
***************************************/

// Event listener for mouse down event.
document.addEventListener("mousedown", (event) => {
    mousedown = true;
});

// Event listener for mouse up event.
document.addEventListener("mouseup", (event) => {
    mousedown = false;
    drawJoystick(canvas.width / 2, canvas.height / 2); // Reset joystick position to center.
});

// Event listener for mouse move event.
document.addEventListener("mousemove", (event) => {
    if (mousedown) {
        yco = event.clientY;
        xco = event.clientX;
        drawJoystick(xco - canvas.offsetLeft, yco - canvas.offsetTop);
    }
});


button_trim_left.addEventListener("mousedown", () => {
	trim_gamma -= 10;
});
button_trim_right.addEventListener("mousedown", () => {
	trim_gamma += 10;
});
