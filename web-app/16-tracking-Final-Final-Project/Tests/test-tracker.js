const axios = require('axios');

const mockUpdate = async () => {
    try {
        const res = await axios.post('http://localhost:3000/api/update', {
            id: "T-100",
            lat: 450, // This is your Y coordinate on the floorplan
            lng: 320  // This is your X coordinate on the floorplan
        });
        console.log("Tracker Ping Sent:", res.status);
    } catch (err) {
        console.error("Error: Is your server running on port 3000?");
    }
};

mockUpdate();