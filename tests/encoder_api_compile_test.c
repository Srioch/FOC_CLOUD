#include "Encoder.h"

int main(void)
{
    Encoder_t encoder = {0};
    EncoderStatus_t status = {0};

    Encoder_ClearDiagnostics(&encoder);
    status = Encoder_GetStatus(&encoder);

    return (Encoder_IsValid(&encoder) == status.valid) ? 0 : 1;
}
