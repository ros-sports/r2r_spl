int8[3] data_int8_static  # Test numeric static array (since it uses numpy arrays)
bool[2] data_bool_static  # Test non-numeric static array
int8[] data_int8_unbounded_dynamic  # Test numeric unbounded array
bool[] data_bool_unbounded_dynamic  # Test non-numeric unbounded array
int8[<=3] data_int8_bounded_dynamic  # Test numeric bounded array
bool[<=2] data_bool_bounded_dynamic  # Test non-numeric bounded array
string data_string  # Test string
string<=3 data_string_bounded  # Test bounded string
wstring data_wstring  # Test wstring
wstring<=3 data_wstring_bounded  # Test bounded wstring
