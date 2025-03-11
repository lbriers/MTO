const alphaElement = document.getElementById('alpha');
const betaElement = document.getElementById('beta');
const gammaElement = document.getElementById('gamma');

// Check if AbsoluteOrientationSensor is supported
if ('AbsoluteOrientationSensor' in window) {
  let sensor = new AbsoluteOrientationSensor({
    frequency: 60, // Adjust frequency as needed (Hz)
    referenceFrame: 'device', // Or 'screen'
  });

  // Request permission (if needed - check browser support)
  if (typeof sensor.requestPermission === 'function') {
    sensor
      .requestPermission()
      .then((permissionState) => {
        if (permissionState === 'granted') {
          startSensor(sensor);
        } else {
          console.warn('Permission to access sensor was denied');
        }
      })
      .catch(console.error);
  } else {
    startSensor(sensor);
  }
} else {
  console.warn('AbsoluteOrientationSensor not supported');
}

function startSensor(sensor) {
  let lastSent = 0;
  const interval = 1000 / 30; // Interval in milliseconds for ~30Hz

  sensor.addEventListener('reading', () => {
    const now = Date.now();
    if (now - lastSent >= interval) {
      // The quaternion values represent the device's orientation
      const quaternion = sensor.quaternion;

      // Convert quaternion to Euler angles (degrees)
      const euler = getEulerAngles(quaternion);
      const alpha = euler.alpha; // Yaw (rotation around Z axis)
      const beta = euler.beta; // Pitch (rotation around X axis)
      const gamma = euler.gamma; // Roll (rotation around Y axis)

      const orientationData = {
        alpha: alpha,
        beta: beta,
        gamma: gamma,
      };

      sendDataToServer(orientationData);
      lastSent = now;
    }
  });

  sensor.addEventListener('error', (event) => {
    console.error('Sensor error:', event.error.message);
  });

  sensor.start();
}

function sendDataToServer(data) {
  fetch('/orientationdata', {
    // Replace '/orientationdata' with your Flask endpoint
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(data),
  })
    .then((response) => {
      if (!response.ok) {
        throw new Error('Network response was not ok');
      }
      console.log('Data sent successfully');
    })
    .catch((error) => {
      console.error('There was a problem sending the data:', error);
    });
}

// Function to convert quaternion to Euler angles (in degrees)
function getEulerAngles(q) {
  const toDegrees = (rad) => (rad * 180) / Math.PI;

  // Convert quaternion to Euler angles (yaw, pitch, roll)
  const sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3]);
  const cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
  const alpha = toDegrees(Math.atan2(sinr_cosp, cosr_cosp)); // Yaw (alpha)

  const sinp = 2 * (q[0] * q[2] - q[3] * q[1]);
  let beta = toDegrees(Math.asin(sinp)); // Pitch (beta)
  if (Math.abs(sinp) >= 1) {
    beta = toDegrees(Math.sign(sinp) * Math.PI / 2); // Use 90 degrees if out of range
  }

  const siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2]);
  const cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
  const gamma = toDegrees(Math.atan2(siny_cosp, cosy_cosp)); // Roll (gamma)

  alphaElement.textContent = alpha.toFixed(2); // Display with 2 decimal places
  betaElement.textContent = beta.toFixed(2); // Display with 2 decimal places
  gammaElement.textContent = gamma.toFixed(2); // Display with 2 decimal places

  return {
    alpha: alpha,
    beta: beta,
    gamma: gamma,
  };
}
