# Исправление проблемы с SSL в base_controller

## Проблема

**Ошибка:** HTTP 400 "The plain HTTP request was sent to HTTPS port"

**Причина:** `CPPHTTPLIB_OPENSSL_SUPPORT` не был определен при компиляции, поэтому `httplib` использовал обычный `Client` вместо `SSLClient` для HTTPS соединений.

## Решение

### 1. Добавлена поддержка OpenSSL в CMakeLists.txt

```cmake
# Find OpenSSL for HTTPS support
find_package(OpenSSL)
if(OpenSSL_FOUND)
  target_link_libraries(async_driver_client OpenSSL::SSL OpenSSL::Crypto)
  target_compile_definitions(async_driver_client PRIVATE CPPHTTPLIB_OPENSSL_SUPPORT)
endif()
```

### 2. Реализованы перегрузки для SSLClient

Добавлены перегрузки функций `sendCommand` и `pollStatus` для работы с `httplib::SSLClient`:

```cpp
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
bool AsyncDriverClient::sendCommand(const geometry_msgs::msg::Twist & cmd, httplib::SSLClient * cli);
void AsyncDriverClient::pollStatus(httplib::SSLClient * cli);
#endif
```

### 3. Исправлена логика создания клиента

Теперь для HTTPS используется `SSLClient`, а для HTTP - обычный `Client`:

```cpp
if (is_https) {
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
    ssl_cli = std::make_unique<httplib::SSLClient>(host, port);
    ssl_cli->enable_server_certificate_verification(tls_verify_);
    // ...
#endif
}
```

## Результат

✅ Сборка успешна  
✅ SSL поддержка включена  
✅ `base_controller` теперь использует `SSLClient` для HTTPS соединений  
✅ Должен получать данные от Symovo API

## Следующие шаги

1. Проверить логи `base_controller` на наличие успешного подключения
2. Проверить `/odom` на наличие реальных данных (x, y, theta)
3. Если данные все еще нулевые, проверить логи на наличие ошибок
