
data = """
[13:52:02.755] Log: Flow: -6.0 4.0
[13:52:02.858] Log: Flow: 0.0 11.0
[13:52:02.956] Log: Flow: 0.0 18.0
[13:52:03.056] Log: Flow: -2.0 24.0
[13:52:03.156] Log: Flow: -4.0 27.0
[13:52:03.255] Log: Flow: -4.0 29.0
[13:52:03.355] Log: Flow: 1.0 31.0
[13:52:03.457] Log: Flow: -4.0 22.0
[13:52:03.564] Log: Flow: -4.0 22.0
[13:52:03.567] Log: Flow: -7.0 21.0
[13:52:03.656] Log: Flow: 0.0 21.0
[13:52:03.756] Log: Flow: -1.0 23.0
[13:52:03.860] Log: Flow: -1.0 23.0
[13:52:03.964] Log: Flow: -1.0 23.0
[13:52:04.041] Log: Flow: -2.0 20.0
[13:52:04.058] Log: Flow: -4.0 16.0
[13:52:04.162] Log: Flow: -3.0 15.0
[13:52:04.256] Log: Flow: 2.0 16.0
[13:52:04.362] Log: Flow: 2.0 16.0
[13:52:04.468] Log: Flow: 2.0 16.0
[13:52:04.527] Log: Flow: 1.0 0.0
[13:52:04.557] Log: Flow: 1.0 0.0
[13:52:04.658] Log: Flow: -2.0 0.0
[13:52:04.758] Log: Flow: 0.0 0.0
[13:52:04.857] Log: Flow: 0.0 -1.0
[13:52:04.957] Log: Flow: 2.0 0.0
[13:52:05.058] Log: Flow: -1.0 0.0
"""

# Extract the flow data
flow_data = []
accumulated_flow = (0.0, 0.0)
for line in data.strip().split('\n'):
    parts = line.split()
    flow_data.append((float(parts[3]), float(parts[4])))
    accumulated_flow = (accumulated_flow[0] + float(parts[3]), accumulated_flow[1] + float(parts[4]))

print(f'Flow data: {flow_data}')
print(f'Accumulated flow: {accumulated_flow}')