# This program is just a test. The final program uses these functions to determine the State of Health of the battery
# The program gets the daily temperature mean history of a battery from zabbix and calculates the Temperature Corrected Years of Battery Life


import sys
import datetime #for timestamp
from pyzabbix.api import ZabbixAPI, ZabbixAPIException #for zabbix api

#function for linear interpolation, this was done because the data for the percentage of battery life is not continuous and differs from battery to battery
def linear_interpolation(x1, y1, x2, y2, x):
    y = (y2 - y1) / (x2 - x1) * (x - x1) + y1
    return y

#data found in https://www.victronenergy.com/upload/documents/Datasheet-GEL-and-AGM-Batteries-EN.pdf
M = 144 #12*12
Z = [(20,1), (30,0.5), (40, 0.25)] #Z is a list of tuples (temperature, percentage of battery life)

#zabbix login data, can be changed
zabbix_url = 'http://test.test.test.test/zabbix'
zabbix_user = 'test'
zabbix_pass = 'test'

#connect to zabbix
zapi = ZabbixAPI(zabbix_url)
zapi.login(zabbix_user, zabbix_pass)

print("Connected to Zabbix API Version %s" % zapi.api_version())

#find host id
host_name = 'test'
hosts = zapi.host.get(filter={"host": host_name}, selectInterfaces=["interfaceid"])
if hosts:
    host_id = hosts[0]["hostid"]
    print("Found host id {0}".format(host_id))

    try:
        itemid = zapi.item.get(hostids=host_id, output='extend', limit='1', search={"key_": "temperature.mean"}) #gets the item id of the temperature.mean item
        id = itemid[0]["itemid"]
        print("Found item id {0}".format(id))
        oneyear = datetime.datetime.now() - datetime.timedelta(days=365) #gets the timestamp of one year ago
        oneyearstamp = datetime.datetime.timestamp(oneyear) #converts the timestamp to unix time
        #gets the history of the temperature.mean item in descending order, sorted by date, from one year ago
        history = zapi.history.get(itemids=id, output='extend', history=0, sortfield='clock', sortorder='DESC', time_from=int(oneyearstamp)) 
        monthly_values = {}
        monthly_means = []
        #gets the monthly mean of the temperature.mean item
        for point in history:
            value = float(point["value"])
            date = datetime.datetime.fromtimestamp(int(point["clock"])).month + datetime.datetime.fromtimestamp(int(point["clock"])).year
            if date not in monthly_values:
                monthly_values[date] = [value]
            else:
                monthly_values[date].append(value)
        for values in monthly_values.values():
            mean = sum(values) / len(values)
            monthly_means.append(mean)
        #for test
        for k in range(9):
            monthly_means.append(25)
        monthly_means.append(30)
        monthly_means.append(31)
        # only 12 months are needed
        if len(monthly_means) > 12:
            del monthly_means[12:]
        monthly_means.sort()
        print(monthly_means)
        if len(monthly_means) == 12:
            groups = []
            mos = []
            #groups the monthly means in groups of temperatures that are close to each other (less than 3 degrees)
            for i in range(12):
                if mos == []:
                    mos.append(monthly_means[i])
                else:
                    a = 0
                    for j in range(len(mos)):
                        if (abs(monthly_means[i] - mos[j]) <= 3):
                            continue
                        else:
                            groups.append(mos)
                            mos = []
                            mos.append(monthly_means[i])
                            a = 1
                            break
                    if a == 0:
                        mos.append(monthly_means[i])
            if mos != []:
                groups.append(mos)
            print(groups)
            k = 0
            # calculates the group mean and the number of months in the group
            # gets the % of battery life for each group
            for i in range(len(groups)):
                leng = len(groups[i])
                a = sum(groups[i]) / len(groups[i])
                if a < Z[0][0] or a > Z[len(Z)-1][0]:
                    k = 1
                    break
                for j in range(len(Z)):
                    if (a < Z[j][0] and j != 0 and a > Z[j-1][0]):
                        a = round(linear_interpolation(Z[j-1][0], Z[j-1][1], Z[j][0], Z[j][1], a), 2)
                        break
                    elif a == Z[j][0]:
                        a = Z[j][1]
                        break
                groups[i] = (a, leng)
            # calculates the Temperature Corrected Years of Battery Life
            if k == 0:
                print(groups)
                d = 0
                for i in range(len(groups)):
                    d += groups[i][1] / groups[i][0]
                l = round(M /d, 1)
                print(l)
            else:
                print("Temperature is too low or too high")
        else:
            print("Not enough data")
    except ZabbixAPIException as e:
        print(e)
        sys.exit()
else:
    print("No hosts found")
