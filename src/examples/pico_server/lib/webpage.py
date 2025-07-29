
def WebPage(gui_state):
    dark_class = "dark" if gui_state.dark_mode else ""
    checked = "checked" if gui_state.dark_mode else ""

    html_head = f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>Neuromechatronics Lab Hand Exo</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    :root {{
      --bg: #f2f2f2;
      --fg: #222;
      --accent: #007aff;
      --card-bg: #fff;
      --log-bg: #eee;
    }}
    body.dark {{
      --bg: #1c1c1e;
      --fg: #f2f2f2;
      --accent: #0a84ff;
      --card-bg: #2c2c2e;
      --log-bg: #3a3a3c;
    }}
    body {{
      margin: 0; padding: 0;
      font-family: -apple-system, BlinkMacSystemFont, sans-serif;
      background-color: var(--bg); color: var(--fg);
    }}
    header {{
      padding: 1rem; display: flex;
      justify-content: space-between; align-items: center;
    }}
    header h1 {{
      font-size: 1.25rem; margin: 0;
    }}
    .card {{
      background-color: var(--card-bg);
      border-radius: 12px;
      padding: 1rem;
      margin: 0.5rem;
      box-shadow: 0 0 6px rgba(0, 0, 0, 0.1);
    }}
    .flex-row {{
      display: flex; flex-wrap: wrap;
      justify-content: center; gap: 1rem;
    }}
    .slider-group {{
      display: flex; justify-content: center;
      gap: 1rem; flex-wrap: wrap;
    }}
    .slider-column {{
      text-align: center;
    }}
    .slider {{
      writing-mode: bt-lr;
      -webkit-appearance: slider-vertical;
      height: 120px;
    }}
    .status-dot {{
      height: 12px; width: 12px;
      border-radius: 50%;
      display: inline-block;
      margin-left: 8px;
    }}
    .connected {{ background-color: #28a745; }}
    .disconnected {{ background-color: #dc3545; }}
    input, select, button {{
      font-size: 0.9rem;
      padding: 6px;
      border-radius: 6px;
      border: 1px solid #ccc;
      background-color: var(--card-bg);
      color: var(--fg);
    }}
    button {{
      background-color: var(--accent);
      color: white;
      border: none;
    }}
    .log {{
      background-color: var(--log-bg);
      font-family: monospace;
      font-size: 0.85rem;
      padding: 1rem;
      white-space: pre-wrap;
      border-radius: 8px;
      max-height: 200px;
      overflow-y: auto;
    }}
    .toggle-dark {{
      position: absolute;
      top: 1rem; right: 1rem;
    }}
    @media (max-width: 768px) {{
      .flex-row {{ flex-direction: column; align-items: center; }}
      .slider-group {{ flex-direction: column; align-items: center; }}
    }}
  </style>
  <script>
    function syncSlider(input, sliderId) {{
      document.getElementById(sliderId).value = input.value;
    }}
    function syncInput(slider, inputId) {{
      document.getElementById(inputId).value = slider.value;
    }}
  </script>
</head>
<body class="{dark_class}">
"""

    html_body = f"""
<header>
  <h1>Neuromechatronics Lab Hand Exo</h1>
  <button class="toggle-dark" onclick="document.getElementById('dark_mode').click()">🌙 Toggle Dark</button>
</header>
<form method="POST">
<input type="checkbox" name="dark_mode" id="dark_mode" style="display:none" onchange="this.form.submit()" {checked}>
<div class="flex-row">
  <div class="card">
    <label>Baudrate:</label><br>
    <input type="number" name="baud" value="{gui_state.baud}"><br><br>
    <button type="submit" name="connect">{'Disconnect' if gui_state.connected else 'Connect'}</button>
    <span class="status-dot {'connected' if gui_state.connected else 'disconnected'}"></span>
  </div>

  <div class="card">
    <label>Operating Mode:</label><br>
    <select name="mode" onchange="this.form.submit()">
      <option value="FREE" {"selected" if gui_state.operating_mode == "FREE" else ""}>FREE</option>
      <option value="GESTURE_FIXED" {"selected" if gui_state.operating_mode == "GESTURE_FIXED" else ""}>GESTURE_FIXED</option>
      <option value="GESTURE_CONTINUOUS" {"selected" if gui_state.operating_mode == "GESTURE_CONTINUOUS" else ""}>GESTURE_CONTINUOUS</option>
    </select>
  </div>
</div>
"""

    # Dynamic slider block
    slider_labels = ["Wrist", "Thumb", "Index", "Middle", "Ring", "Pinky"]
    slider_group = '<div class="card"><div class="slider-group">\n'
    for i, label in enumerate(slider_labels):
        val = gui_state.motor_values[i]
        slider_group += f"""
  <div class="slider-column">
    <label>{label}</label><br>
    <input type="range" class="slider" id="slider{i}" name="motor{i}" min="-90" max="90" value="{val}"
           oninput="syncInput(this, 'input{i}')" 
           onmouseup="this.form.submit()" ontouchend="this.form.submit()"><br>
    <input type="number" id="input{i}" name="angle{i}" value="{val}" min="-90" max="90"
           oninput="syncSlider(this, 'slider{i}')" onchange="this.form.submit()">
  </div>
"""
    slider_group += '</div></div>\n'

    # Direct message and log
    direct_input = """
<div class="card">
  <label>Custom Serial Command:</label><br>
  <input type="text" name="direct_msg" placeholder="Enter command..." 
         onkeydown="if(event.key==='Enter'){this.form.submit();return false;}">
  <button type="submit">Send</button>
</div>
"""

    log_card = f"""
<div class="card">
  <label>Log:</label>
  <div class="log">{gui_state.log_text}</div>
</div>
</form>
</body>
</html>
"""

    return html_head + html_body + slider_group + direct_input + log_card
