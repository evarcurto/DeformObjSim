# DeformObjSim

Environment installation

- Creation of a new conda environment:

```python
conda create --name BulletEnv python=python3.8
```

- Install PyBullet in BulletEnv using pip:

```python
pip3 install pybullet --upgrade --user
```

- Clone bullet3 repository to the BulletEnv

```python
git clone 'https://github.com/bulletphysics/bullet3.git'
```

Running simulator

- Activation of BulletEnv environment:
```python
conda activate BulletEnv
```
- Clone DeformObjSim repository
```python
git clone 'https://github.com/evarcurto/DeformObjSim.git'
```
- Change directory to DeformObjSim
```python
cd DeformObjSim
```
- Run main script
```python
python main.py
```
