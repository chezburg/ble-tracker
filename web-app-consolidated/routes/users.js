var express = require('express');
var router = express.Router();

// Mock database to store users temporarily
// Note: This resets every time you restart the server!
const users = [];

/* GET users listing. */
router.get('/', function(req, res, next) {
  res.send('respond with a resource');
});

/* POST: Register a new user */
router.post('/api/register', async (req, res) => {
  try {
    const { username, password, role } = req.body;

    // 1. Basic validation
    if (!username || !password) {
      return res.status(400).json({ message: "Username and password are required." });
    }

    // 2. Check if user already exists
    const userExists = users.find(u => u.username === username);
    if (userExists) {
      return res.status(409).json({ message: "User already exists!" });
    }

    // 3. Save user (In a real app, use bcrypt to hash the password here!)
    const newUser = { username, password, role };
    users.push(newUser);

    console.log("New user registered:", newUser);
    console.log("Current user list:", users);

    // 4. Send success response
    res.status(201).json({
      message: "Account created successfully for " + username
    });

  } catch (error) {
    console.error("Registration Error:", error);
    res.status(500).json({ message: "Internal server error." });
  }
});

module.exports = router;