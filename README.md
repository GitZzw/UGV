# catkin_ws
## Steps
1. catkin_make
cd catkin_ws
catkin_make


# nanodet
## Steps
1. Install NVIDIA driver & nvcc

2. Install Anaconda3 
Anaconda Download: official site https://www.anaconda.com/products/individual#Downloads

3. create vitual env & activate
conda create -n zzw python=3.8 -y
conda activate zzw

4. Install pytorch
conda install pytorch torchvision cudatoolkit=11.1 -c pytorch -c conda-forge

5. Install requirement
pip install Cython termcolor rospkg numpy tensorboard pycocotools matplotlib pyaml opencv-python tqdm pytorch-lightning torchmetrics

6. Setup Nanodet
cd nanodet 
python setup.py develop

