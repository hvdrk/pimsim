import numpy as np
import sys
import os

# 커맨드라인 인자 처리
if len(sys.argv) != 2:
    print("Usage: python gen.py <row_size>")
    sys.exit(1)

try:
    row_size = int(sys.argv[1])
except ValueError:
    print("row_size must be an integer")
    sys.exit(1)

# gen.py 파일이 위치한 디렉토리 기준으로 저장 경로 설정
base_dir = os.path.dirname(os.path.abspath(__file__))

inner_size = 128 * 16 * row_size
outer_size = 128 * 16 * row_size

data_type = np.uint32

k1 = np.random.permutation(inner_size).astype(data_type)
v1 = np.random.randint(0, 64, size=inner_size, dtype=data_type)
inner_rel = np.array([[k, v] for k, v in zip(k1, v1)], dtype=data_type)
np.save(os.path.join(base_dir, "inner"), inner_rel)

print(f"inner_rel : (shape: {inner_rel.shape}):")
print(inner_rel)

k2 = np.random.choice(k1, size=outer_size, replace=False)
v2 = np.random.randint(0, 64, size=outer_size, dtype=data_type)
outer_rel = np.array([[k, v] for k, v in zip(k2, v2)], dtype=data_type)
np.save(os.path.join(base_dir, "outer"), outer_rel)

print(f"outer_rel : (shape: {outer_rel.shape}):")
print(outer_rel)

v2_sum = np.sum(v2)
print(v2_sum)
result = np.array([v2_sum, 0, 0, 0, 0, 0, 0, 0], dtype=data_type)
np.save(os.path.join(base_dir, "result"), result)

print(f"result : {result}")