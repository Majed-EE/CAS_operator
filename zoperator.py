#operator
import requests

url = "http://127.0.0.1:5000/receive"  # Server endpoint

print("operator side")
for x in range (4):
    user_input=str(input("user input"))
    data_to_send = {"user_input": user_input}  # Data to send
    response = requests.post(url, json=data_to_send)

    print(f"Server response: {response.json()}")