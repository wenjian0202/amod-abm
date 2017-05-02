import requests

def create_url(ghost, gport, olat, olon, dlat, dlon):
    return "http://{0}:{1}/route/v1/driving/{2},{3};{4},{5}?alternatives=false&steps=false".format(
        ghost, gport, olat, olon, dlat, dlon)
 
def call_url(url):
    """ Send the request and get the response in Json format """
    # print(url)
    try:
        response = requests.get(url)
        json_response = response.json()
        code = json_response['code']
        if code == 'Ok':
            return (json_response, True)
        else:
            print("Error: %s" % (json_response['message']))
            return (json_response, False)
    except Exception as err:
        print("Failed: %s" % (url))