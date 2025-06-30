
To generate new dataset,
```
python ./data/join/gen_join.py <row_size>
```
Total dataset size is <row_size> * 32KB

To simulate,
```
./sim --gtest_filter=PIMBenchFixture.join
```
