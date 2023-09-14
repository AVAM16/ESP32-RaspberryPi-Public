# This program is just a test. The final program is using BluePy instead of Bleak.
# The objective of this program is to connect to a BLE-Server and interact with it.


import asyncio  #mandatory package to use bleak     #allows use of coroutines
import bleak    #client agnostic BLE library for Python #can be used with bluetooth stack
import struct  

# UUIDs for the service and its characteristics
uuid_custom_service = 'test'
uuid_read_characteristic = 'test'
uuid_write_characteristic = 'test'

# coroutine to connect to individual BLE-Server
async def connect(address):
    try:
        async with bleak.BleakClient(address) as client:
            #asks device for temperature data sends "t" to device
            output = ("t").encode()
            await client.write_gatt_char(uuid_write_characteristic, output)
            #reads temperature data from device
            readvalue = await client.read_gatt_char(uuid_read_characteristic)
            #the device sends a struct with 2 floats, num1 is the temperature and num2 is 0
            num1, num2 = struct.unpack('ff', readvalue)
            print(num1, num2)
            #asks device to sleep sends "s" to device
            output = ("s").encode()
            await client.write_gatt_char(uuid_write_characteristic, output)
    except bleak.exc.BleakDeviceNotFoundError:
        print("Could not connect to BLE-Server. Device not found.")
    await asyncio.sleep(1)

# main function
async def main():
    #due to an issue with this Raspberry Pi's BlueZ stack, the services array is empty
    devices = await bleak.BleakScanner.discover()  # Standard Scanning duration is 5 seconds
    tasks = []
    for d in devices:
        # found devices by name just for testing, should be changed to address
        if str(d.details.get('props').get('Name')).startswith('NimBLE-Server'):
            address = d.details.get('props').get('Address') # get address of device
            tasks.append(asyncio.create_task(connect(str(address)))) # create coroutine for each device
    count = 0
    # wait for all coroutines to finish
    for t in tasks:
        count += 1
        await t
    print(f"Finished {count} tasks.")

    

# run main function
asyncio.run(main())
