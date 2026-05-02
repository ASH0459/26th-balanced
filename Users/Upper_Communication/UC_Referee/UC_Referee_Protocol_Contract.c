#include "UC_Referee.h"

_Static_assert(sizeof(ext_game_status_t) == 11U, "0x0001 size mismatch");
_Static_assert(sizeof(ext_game_result_t) == 1U, "0x0002 size mismatch");
_Static_assert(sizeof(ext_game_robot_HP_t) == 16U, "0x0003 size mismatch");
_Static_assert(sizeof(ext_event_data_t) == 4U, "0x0101 size mismatch");
_Static_assert(sizeof(ext_referee_warning_t) == 3U, "0x0104 size mismatch");
_Static_assert(sizeof(ext_dart_remaining_time_t) == 3U, "0x0105 size mismatch");
_Static_assert(sizeof(ext_game_robot_status_t) == 13U, "0x0201 size mismatch");
_Static_assert(sizeof(ext_power_heat_data_t) == 14U, "0x0202 size mismatch");
_Static_assert(sizeof(ext_game_robot_pos_t) == 12U, "0x0203 size mismatch");
_Static_assert(sizeof(ext_buff_t) == 8U, "0x0204 size mismatch");
_Static_assert(sizeof(ext_robot_hurt_t) == 1U, "0x0206 size mismatch");
_Static_assert(sizeof(ext_shoot_data_t) == 7U, "0x0207 size mismatch");
_Static_assert(sizeof(ext_bullet_remaining_t) == 8U, "0x0208 size mismatch");
_Static_assert(sizeof(ext_rfid_status_t) == 5U, "0x0209 size mismatch");
_Static_assert(sizeof(ext_student_interactive_header_data_t) == 6U, "0x0301 header size mismatch");
