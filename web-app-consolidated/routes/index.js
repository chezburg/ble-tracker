/** A
 * SSET TRACKER BACKEND - ROUTE CONTROLLER
 * This file manages user accounts, login security, and the
 * tracking data for hospital equipment.
 */

var express = require('express');
var router = express.Router();

// External tools for security and tokens
const bcrypt = require('bcryptjs');
const jwt = require('jsonwebtoken');

//Security Key for the backend (Do not change)
const JWT_SECRET = "67";

// MIDDLEWARE: Verifies that a user is logged in before allowing access
const authenticateToken = (req, res, next) => {
  const token = req.headers['authorization']?.split(" ")[1];
  if (!token) return res.status(401).json({ message: "No token provided" });
  jwt.verify(token, JWT_SECRET, (err, user) => {
    if (err) return res.status(403).json({ message: "Invalid token" });
    req.user = user;
    next();
  });
};

/* --- TEMPORARY SYSTEM DATA --- */
let users = [];
let latestTrackerData = { x: 0, y: 0, floor: 0, lastUpdated: new Date() };

let locationHistory = [
  { id: "T-100", name: "Red Truck", location: "North Wing - Room 202", lastUpdated: new Date("2026-03-25T10:00:00"), floor: "North"},
  { id: "T-100", name: "Red Truck", location: "North Wing - Hallway B", lastUpdated: new Date("2026-03-25T09:30:00"), floor: "North"},
  { id: "T-100", name: "Red Truck", location: "Loading Dock", lastUpdated: new Date("2026-03-25T09:00:00"), floor: "South" },
  { id: "T-100", name: "Red Truck", location: "Loading Dock", lastUpdated: new Date("2026-03-10T09:00:00"), floor: "South" },
  { id: "T-200", name: "WheelChair", location: "North Wing - Room 202", lastUpdated: new Date("2026-03-03T10:00:00"),floor: "North" },
  { id: "T-200", name: "WheelChair", location: "North Wing - Hallway B", lastUpdated: new Date("2026-03-14T09:30:00"), floor: "North"  },
  { id: "T-200", name: "WheelChair", location: "Loading Dock", lastUpdated: new Date("2026-03-18T09:00:00"), floor: "North"  },
  { id: "T-200", name: "WheelChair", location: "Loading Dock", lastUpdated: new Date("2026-03-10T09:00:00"), floor: "South"  }
];

let assets = [
  {
    id: "T-100",
    name: "Red Truck",
    category: "crash carts", // Matches 'Crash Carts' filter
    floor: "north",          // Matches 'North Wing' filter
    location: "Room 202",    // For the Detail Panel
    lastUpdated: new Date()
  },
  {
    id: "T-200",
    name: "Wheelchair",
    category: "wheelchairs", // Matches 'Wheelchairs' filter (added 's')
    floor: "south",          // Matches 'South Wing' filter
    location: "Hallway B",   // For the Detail Panel
    lastUpdated: new Date()
  }
];

// MIDDLEWARE: Restricts actions based on user role (e.g., IT vs General)
const authorizeRole = (roleRequired) => {
  return (req, res, next) => {
    authenticateToken(req, res, () => {
      // Force both to lowercase for a safe comparison
      const userRole = req.user.role ? req.user.role.toLowerCase() : "";
      // Check if user is the specific role OR a general admin
      if (userRole === roleRequired.toLowerCase() || userRole === "admin") {
        next();
      } else {
        console.log(`Access Denied. User Role: ${userRole}, Required: ${roleRequired}`);
        res.status(403).json({ message: "Forbidden: Higher privileges required." });
      }
    });
  };
};

/* --- USER AUTHENTICATION ROUTES --- */

// POST: Register a new user and hash their password
router.post('/api/register', async (req, res) => {
  const { username, password, role } = req.body;
  // 1. CHECK IF USER EXISTS
  const userExists = users.some(u => u.username === username);
  if (userExists) {
    // 409 Conflict is the standard status code for duplicate data
    return res.status(409).json({ message: "Username already taken!" });
  }
  // 2. PROCEED IF UNIQUE
  const hashedPassword = await bcrypt.hash(password, 10);
  users.push({ username, password: hashedPassword, role: role });
  res.status(201).json({ message: "User created!" });
});

// POST: Log in a user and return a security token
router.post('/api/login', async (req, res) => {
  const { username, password } = req.body;
  const user = users.find(u => u.username === username);
  // Check if user exists first
  if (!user) {
    return res.status(404).json({ message: "User not found. Please register." });
  }
  // Then check password
  const isMatch = await bcrypt.compare(password, user.password);
  if (isMatch) {
    const token = jwt.sign({ username: user.username, role: user.role }, JWT_SECRET);
    res.json({ token, role: user.role });
  } else {
    res.status(401).json({ message: "Invalid password." });
  }
});

/* --- ASSET MANAGEMENT ROUTES --- */

// GET: Retrieve location history for a specific asset I
router.get('/api/history/:id', authenticateToken, (req, res) => {
  const { id } = req.params;
  const days = req.query.days ? parseInt(req.query.days) : 0;
  // Filter by ID
  let results = locationHistory.filter(h => h.id === id);
  if (days > 0) {
    const cutoff = new Date();
    cutoff.setDate(cutoff.getDate() - days);
    results = results.filter(h => new Date(h.lastUpdated) >= cutoff);
  }

  //Sort by newest first so the top of the table is the most recent
  results.sort((a, b) => new Date(b.lastUpdated) - new Date(a.lastUpdated));
  res.json(results);
});

// GET: Render the main dashboard page
router.get('/', function(req, res, next) {
  res.render('index', { title: 'Asset Tracker System' });
});

//Search route
// This lets the user search for assets.
// Example: http://localhost:3000/api/search?name=Truck
router.get('/api/search', authenticateToken, (req, res) => {

  // Grabs the name after the first equal sign and makes sure its lower case
  // ?: represents if/else, if the name is entered use that, else use empty ""
  const query = req.query.name ? req.query.name.toLowerCase() : "";
  // Filter() goes through all the assets in the asset variable
  //.includes(query) looks for the name and stores the information if found in asset
  const results = assets.filter(a => a.name.toLowerCase().includes(query));
  //If data found, sends back data, or it prints an error
  if (results.length > 0) {
    res.json(results);
  } else {
    res.status(404).json({ message: "No asset found with that name." });
  }
});

// GET: List all currently tracked assets
router.get('/api/assets', authenticateToken, (req, res) => {
  res.json(assets);
});

// DELETE: Remove an asset from the list (Requires IT permissions)
router.delete('/api/assets/:id', authorizeRole("it"),  (req, res) => {
  //stores the object we want to delete
  const {id} = req.params;
  //Checks how long the 'before deletion' object is
  const initialLength = assets.length;
  // Keep only the assets that DON'T match the ID provided
  assets = assets.filter(a => a.id !== id);
  //Checks if the current asset length is not the same as the previous asset length
  if (assets.length < initialLength) {
    res.json({message: `Asset ${id} deleted successfully.`});
  } else {
    res.status(404).json({message: "Asset not found."});
  }
})

// Adding a new asset to the list
router.post('/api/assets', authorizeRole("it"), (req, res) => {
  const { id, name } = req.body;
  //Basic Validation: Ensure the user sent an ID and a Name
  if (!id || !name) {
    return res.status(400).json({ message: "ID and Name are required." });
  }
  //Prevent Duplicates: Check if the ID already exists
  const existing = assets.find(a => a.id === id);
  if (existing) {
    return res.status(409).json({ message: "An asset with this ID already exists." });
  }
  // POST: Add a new asset to the tracking system (Requires IT permissions)
  const newAsset = {
    id: id,
    name: name,
    lat: 0, // Default position until the tracker sends real data
    lng: 0,
    lastUpdated: new Date()
  };
  //Save to the asset list
  assets.push(newAsset);
  res.status(201).json({ message: "Asset added successfully!", asset: newAsset });
});
/* --- TRACKER INTEGRATION ROUTES --- */

// POST: Receive live tracker data (called by ESP32-C3)
router.post('/data', (req, res) => {
  const { x, y, floor } = req.body;
  
  // 1. Update global latest variable for simple Map.html
  latestTrackerData = { 
    x: parseFloat(x) || 0, 
    y: parseFloat(y) || 0, 
    floor: parseInt(floor) || 0, 
    lastUpdated: new Date() 
  };

  // 2. Map tracker data to asset T-100 (demo)
  const asset = assets.find(a => a.id === "T-100");
  if (asset) {
    asset.lat = latestTrackerData.y; // Map XY to internal lat/lng
    asset.lng = latestTrackerData.x;
    asset.floor = latestTrackerData.floor;
    asset.lastUpdated = latestTrackerData.lastUpdated;
    
    // 3. Log to history if position changed significantly (optional, but good)
    locationHistory.unshift({
      id: asset.id,
      name: asset.name,
      location: `(X: ${asset.lng.toFixed(1)}, Y: ${asset.lat.toFixed(1)})`,
      lastUpdated: asset.lastUpdated,
      floor: `Floor ${asset.floor}`
    });
    // Keep history manageable
    if (locationHistory.length > 100) locationHistory.pop();
  }

  console.log(`[TRACKER] Updated T-100 to (${x}, ${y}) Floor ${floor}`);
  res.json({ status: "ok" });
});

// GET: Retrieve the most recent tracker position (called by Map.html)
router.get('/position', (req, res) => {
  res.json(latestTrackerData || { x: 0, y: 0, floor: 0 });
});

module.exports = router;
