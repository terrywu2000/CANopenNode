file(MAKE_DIRECTORY "${WORK}")
file(WRITE "${WORK}/runner.json"
    "{\"interface_name\":\"CAN1\",\"model_path\":\"${MODEL}\",\"persistence_path\":\"${WORK}/device.lss.json\"}")
execute_process(COMMAND "${APP}" --lib "${LIB}" --config "${WORK}/runner.json" --duration 1
    RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE errors TIMEOUT 10)
if(NOT result EQUAL 0 OR NOT output MATCHES "modsim epoch=" OR NOT output MATCHES "accepted=[1-9]"
    OR NOT errors MATCHES "Model stopped:" OR NOT errors MATCHES "cleanup complete")
    message(FATAL_ERROR "modsim smoke failed (${result})\n${output}\n${errors}")
endif()
execute_process(COMMAND "${APP}" --lib "${LIB}" --config "${WORK}/runner.json" --duration 0
    RESULT_VARIABLE invalid OUTPUT_VARIABLE output ERROR_VARIABLE errors TIMEOUT 5)
if(invalid EQUAL 0)
    message(FATAL_ERROR "modsim accepted invalid duration")
endif()
file(WRITE "${WORK}/motor.json" "{\"position_gain\":30,\"action_delay_us\":20000}")
execute_process(COMMAND "${APP}" --lib "${LIB}" --config "${WORK}/runner.json"
    --motor "${WORK}/motor.json" --duration 1
    RESULT_VARIABLE configured OUTPUT_VARIABLE output ERROR_VARIABLE errors TIMEOUT 10)
if(NOT configured EQUAL 0 OR NOT errors MATCHES "accepted=[1-9]")
    message(FATAL_ERROR "configured modsim failed (${configured})\n${output}\n${errors}")
endif()
foreach(invalid_motor IN ITEMS "{\"period_us\":-1}" "{\"initial_position\":4294967296}"
    "{\"position_gain\":0}" "{\"unknown\":1}" "{\"fault_code\":65536}" "{\"period_us\":1.5}")
    file(WRITE "${WORK}/invalid-motor.json" "${invalid_motor}")
    execute_process(COMMAND "${APP}" --lib "${LIB}" --config "${WORK}/runner.json"
        --motor "${WORK}/invalid-motor.json" --duration 1
        RESULT_VARIABLE invalid OUTPUT_VARIABLE output ERROR_VARIABLE errors TIMEOUT 5)
    if(invalid EQUAL 0 OR NOT errors MATCHES "Error:")
        message(FATAL_ERROR "modsim failed to reject invalid motor parameters: ${invalid_motor}")
    endif()
endforeach()
