图像中的文字是：
![alt text](image.png)
```
github.com/ZATiTech/autoware.universe.openplanner.git
```

![alt text](image-1.png)

https://github.com/autowarefoundation/autoware.git

mkdir -p ./autoware-universe/src
cd ~/autoware-universe
git clone https://github.com/autowarefoundation/autoware.git src/autoware


git@github.com:autowarefoundation/autoware.git

git@github.com:ZATiTech/autoware.universe.openplanner.git

![alt text](image-2.png)

vcs import src < autoware.repos

`source /opt/ros/galactic/setup.bash`  
rosdep install -y --from-paths src --ignore-src --rosdistro foxy

![alt text](image-3.png)
