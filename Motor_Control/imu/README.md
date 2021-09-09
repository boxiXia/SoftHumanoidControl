# To compile
```
sudo g++ ./uart_905_demo.cpp -o imu.out -pthread 
```

# To run
```
./imu.out
```

# Common problem
## Serial port terminal > Cannot open /dev/ttyS0: Permission denied
First check if you are a member of that group:
```
groups ${USER}
```
this will list all the groups you belong to. If you don't belong to the dialout grup then add yourself to it, for example:
```
sudo gpasswd --add ${USER} dialout
```
You then need to log out and log back in again for it to be effective. Then see if it fixes your proble