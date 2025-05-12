const express = require('express');
const cors = require('cors');
const mongoose = require('mongoose');

const app = express();
app.use(cors()); // <- This enables CORS
app.use(express.json());
const PORT=4000;

mongoose.connect('mongodb://localhost:27017/ros_missions', {
  useNewUrlParser: true,
  useUnifiedTopology: true,
});

const stationSchema = new mongoose.Schema({
  name: String,
  x: Number,
  y: Number,
  timestamp: { type: Date, default: Date.now }
});

const Station = mongoose.model('Station', stationSchema);

// server/index.js (add/update)
app.post('/stations', async (req, res) => {
  const { name, x, y } = req.body;
  console.log(req.body);

  if (!name || x === undefined || y === undefined) {
    return res.status(400).json({ error: 'Missing data' });
  }

  try {
    const station = new Station({ name, x, y });
    await station.save();
    res.json({ message: ' Station saved', station });
  } catch (err) {
    res.status(500).json({ error: ' Failed to save station' });
  }
});
app.get('/stations', async (req, res) => {
  try {
    const stations = await Station.find(); // or .sort({ createdAt: 1 })
    res.json(stations);
  } catch (err) {
    res.status(500).json({ error: 'Failed to fetch stations' });
  }
});



app.listen(PORT, () => {
  console.log(`🚀 Server running on http://localhost:${PORT}`);
});
