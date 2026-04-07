const express = require("express");
const app = express();

app.use(express.json());

let latest = {x:0, y:0, floor:0};

app,post("/data", (rqe,res) => {
    console.log("Incoming", req.body);

    const {x,y,floor} = req.body;

    latest = {x,y,floor};

    res.json({status: "ok"});

});

app.get("/position", (req,res) => {
    res.json(latest);
});

app.listen(3000, () => console.log("Server Running"));

