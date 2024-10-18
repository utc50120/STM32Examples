#ifndef __JKIT_FND_H
#define __JKIT_FND_H

void JKIT_FND_error(char error_number);
void JKIT_FND_process(void);

/* number = (sign_minus:1)-999 ~ (sign_minus:0)9999, display = -99.9 ~ 999.9 */
void JKIT_FND_number(int number, char sign_minus);

#endif /* __JKIT_FND_H */
